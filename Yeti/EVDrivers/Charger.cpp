/*
 * Charger.cpp
 *
 *  Created on: 08.03.2021
 *      Author: cornelius
 *
 */

#include "Charger.h"
#include "RemoteControl.h"

#include <math.h>
#include <string.h>

#include "EVConfig.h"

Charger::Charger(ControlPilot &_control_pilot, Display *_display,
                 ADE7978 &_power_meter) :
    Task("Charger", 2048 * 4),
    display(_display),
    power_meter(_power_meter),
    control_pilot(_control_pilot) {

    maxCurrent = 0;
    hardCurrentLimit = 6;
    lastPwmUpdate = 0;
    authorized = false;
    remote_control = NULL;
    controlMode = ControlMode::None;

    simulation = false;
    sim_CPVoltage = 12.;
    sim_ppResistor = 220.;
    sim_currents[0] = 0.;
    sim_currents[1] = 0.;
    sim_currents[2] = 0.;
    sim_currents[3] = 0.;
    sim_voltages[0] = 230.;
    sim_voltages[1] = 230.;
    sim_voltages[2] = 230.;
    sim_freq[0] = 50.;
    sim_freq[1] = 50.;
    sim_freq[2] = 50.;
    sim_rcdCurrent = 0.;
    sim_diodeFail = false;
    sim_errorE = false;
    sim_wattHr[0] = 0.;
    sim_wattHr[1] = 0.;
    sim_wattHr[2] = 0.;

    sim_powermeter_lastTick = HAL_GetTick();

    control_pilot.disable();

    currentState = EvseState::Disabled;

    configMutex = osMutexNew(&configMutex_attributes);
    stateMutex = osMutexNew(&stateMutex_attributes);
}

Charger::~Charger() { control_pilot.pwmF(); }

void Charger::main() {

    int cnt = 0;

    currentState = EvseState::Disabled;

    // Enable CP output
    control_pilot.enable();

    while (true) {
        osDelay(50);

        if (cnt++ % 5 == 0) {
            updatePowerMeter();
            checkOverCurrent();
            updateDisplay();
        }

#ifdef FINE_GRAIN_DEBUG_PRINTF
	if (cnt % 20 == 0) {
            printf("CMode %i\n", (int)getControlMode());
        }

#endif


        osMutexWait(stateMutex, osWaitForever);

        // Run low level state machine update
        auto cp_events = control_pilot.runStateMachine();

        // if control_mode == None|High we run the actual charging logic here,
        // else if it is Low, we just proxy control_pilot to the protobuf
        // interface Process all low level events in the queue (i.e. run all
        // actions that are triggered on a specific event)
        if (getControlMode() == ControlMode::None ||
            getControlMode() == ControlMode::High) {
            while (!cp_events.empty()) {
                auto cp_event = cp_events.front();
                cp_events.pop();
#ifdef CHARGER_CPP_ENABLE_PRINTF
                char s[50];
                evseStateToString(s, currentState);
                printf("State %s CP Event received: %s\n", s,
                       control_pilot.eventToString(cp_event));
#endif
                // Process all event actions that are independent of the current
                // state
                processCPEventsIndependent(cp_event);

                // Process all events that depend on the current state
                processCPEventsState(cp_event);
            }

            // Run our own state machine update (i.e. run everything that needs
            // to be done on regular intervals independent from events)
            runStateMachine();
        } else {
            // we just proxy the ControlPilot interface to protobuf
            while (!cp_events.empty()) {
                auto cp_event = cp_events.front();
                cp_events.pop();

                // Forward signals to RemoteControl
                if (remote_control)
                    remote_control->event(cp_event);
            }
        }

        osMutexRelease(stateMutex);
    }
}

void Charger::runStateMachine() {
    switch (currentState) {
    case EvseState::Disabled:
        control_pilot.pwmF();
        break;

    case EvseState::Idle:
        // make sure we signal availability to potential new cars
        control_pilot.pwmOff();
        Authorize(false, "");
        power_meter.resetWATTHR();
        sim_wattHr[0] = 0.;
        sim_wattHr[1] = 0.;
        sim_wattHr[2] = 0.;
        // reset 5 percent flag for this session
        /*if (dc_mode)
            use_5percent = true;
        else if (ac_start_with_5percent)
            use_5percent = ac_start_with_5percent;
        else
            use_5percent = false;*/
        break;

    case EvseState::WaitingForAuthentication:

        // Explicitly do not allow to be powered on. This is important
        // to make sure control_pilot does not switch on relais even if
        // we start PWM here
        control_pilot.allowPowerOn(false);

        // retry on Low level etc until SLAC matched.
        ISO_IEC_Coordination();

        // we get Auth (maybe before SLAC matching or during matching)
        if (getControlMode() == ControlMode::None || getAuthorization()) {
            currentState = EvseState::ChargingPausedEV;
        }

        break;

    case EvseState::Charging:
        /*if (use_5percent)
            control_pilot.pwmOn(0.05);
        // FIXME: only update every 5 seconds!
        else*/
        control_pilot.pwmOn(ampereToDutyCycle(getMaxCurrent()));
        control_pilot.allowPowerOn(true);
        break;

    case EvseState::ChargingPausedEV:
        /*if (use_5percent)
            control_pilot.pwmOn(0.05);
        // FIXME: only update every 5 seconds!
        else*/
        control_pilot.pwmOn(ampereToDutyCycle(getMaxCurrent()));
        control_pilot.allowPowerOn(true);
        break;

    case EvseState::ChargingPausedEVSE:
        control_pilot.pwmOff();
        break;

    case EvseState::Finished:
        if (getControlMode() == ControlMode::None) {
            currentState = EvseState::Idle;
        }
        // We are temporarily unavailable to avoid that new cars plug in
        // during the cleanup phase. Re-Enable once we are in idle.
        control_pilot.pwmF();
        break;

    case EvseState::Error:
        control_pilot.pwmOff();
        // Do not set F here, as F cannot detect unplugging of car!
        break;

    case EvseState::Faulted:
        control_pilot.pwmF();
        break;
    }
}

void Charger::processCPEventsState(ControlPilot::Event cp_event) {
    switch (currentState) {

    case EvseState::Idle:
        if (cp_event == ControlPilot::Event::CarPluggedIn) {
            // FIXME: Enable SLAC sounding here, as sounding is forbidden before
            // CP state B
            // FIXME: Set cable current limit from PP resistor value for this
            // session e.g. readPPAmpacity();
            currentState = EvseState::WaitingForAuthentication;
        }
        break;

    case EvseState::WaitingForAuthentication:
        // FIXME: in simplified mode, we get a car requested power here directly
        // even if PWM is not enabled yet... this should be fixed in control
        // pilot logic? i.e. car requests power only if pwm was enabled? Could
        // be a fix, but with 5% mode it will not work.
        if (cp_event == ControlPilot::Event::CarRequestedPower) {
            // FIXME
        }
        break;

    case EvseState::Charging:
        if (cp_event == ControlPilot::Event::CarRequestedStopPower) {
            currentState = EvseState::ChargingPausedEV;
        }
        break;

    case EvseState::ChargingPausedEV:
        if (cp_event == ControlPilot::Event::CarRequestedPower) {
            currentState = EvseState::Charging;
        }
        break;
    default:
        break;
    }
}

void Charger::processCPEventsIndependent(ControlPilot::Event cp_event) {

    switch (cp_event) {
    case ControlPilot::Event::CarUnplugged:
        // FIXME: Stop SLAC sounding here
        currentState = EvseState::Finished;
        break;
    case ControlPilot::Event::Error_E:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_E;
        break;
    case ControlPilot::Event::Error_DF:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_DF;
        break;
    case ControlPilot::Event::Error_Relais:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_Relais;
        break;
    case ControlPilot::Event::Error_RCD:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_RCD;
        break;
    case ControlPilot::Event::Error_VentilationNotAvailable:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_VentilationNotAvailable;
        break;
    case ControlPilot::Event::Error_OverCurrent:
        currentState = EvseState::Error;
        errorState = ErrorState::Error_OverCurrent;
        break;
    default:
        break;
    }
}

void Charger::ISO_IEC_Coordination() {
    /*
     *
     * ISO15118 / IEC61851 coordination
     * --------------------------------
     *
     * Hi->Lo settings:
     * ----------------
     * dc_mode: true (DC), false (AC)
     * ac_start_with_5percent: true (start with 5% in AC), false
     * (use only nominal PWM). Note that the internal flag
     * use_5percent can be disabled by Yeti e.g. on failover.
     *
     * ac_enforce_hlc_with_eim: true (non-std; don't shortcut to
     * nominal if auth arrives before matching), false: conform
     * to norm
     *
     * Hi provides the following information to Lo:
     * --------------------------------------------
     * slac state: disabled, unmatched, matching, matched
     *
     * Lo can trigger the following commands on Hi:
     * --------------------------------------------
     * slac_enable(true/false): enable/disable SLAC to start
     * matching sessions slac_dlink_terminate: if e.g. car
     * unplugs or error E is detected from car Yeti can
     * terminate the Link.
     *
     * Disabled: F
     * Idle: A1
     * WaitingForAuthAndOrMatching/Setup: B1,B2,E,F
     * Charging: C2,D2,E,F
     * PausedByEV: B1,E,F
     * PausedByEVSE: B1,C1,D1,E,F
     * Finished: A1
     * Faulted: F
     * Error: F, E
     *
     * 1) ABCDEF state (PWM output, relais an aus etc)
     * 2) Logical states / coordination (Disabled ... Charging
     * etc) 3) SLAC 4) V2G
     *
     * ABCDE <> coordiator <> GreenPHY <> V2G
     *
     *
     * terminate: fallback nominal BX1B?
     *
     * Hi can trigger the following commands on Lo:
     * --------------------------------------------
     * slac_dlink_error: ISO15118-3 Table 6 says "restart
     * matching by CP transition through E". They probably meant
     * a BFB sequence, see [V2G3-M07-04]. Initialize a BFB to
     * WaitingForAuth. IEC requires auth to be kept, so we keep
     * the authorized flag.
     *
     *                   if (use_5percent == true):
     *                   SLAC sends this command to LO _after_
     * it left the logical network [V2G3-M07-07]
     *
     *                   -> BFB to WaitingForAuth (as this
     * triggers retries etc). Note that on AC this leads to
     * immediate progression to BC as auth is still valid and
     * auth comes before matching started.
     *
     *                   if (use_5percent == false):
     *                   we choose to ignore it here as leaving
     * the network is handled in SLAC stack [V2G3-M07-12]
     *
     *                   Note: don't use dlink_error on AC with
     * 5% when e.g. tcp connection dies. This will lead to
     * endless retries and no fallback to nominal.
     *
     * (not needed here) slac_dlink_pause: pause is sent to both
     * ISO and IEC stack, so no communication is needed during
     * entering pause mode. Unclear how to exit pause mode (i.e.
     * wake-up sequencing)
     *
     *
     *
     * dc_mode == false && ac_use_5percent == false: wait for
     * auth and just continue. ignore matching/matched events.
     * This mode is typically BC, but matching can occur at any
     * time during the session (initiated by car).
     *
     * dc_mode == false && ac_use_5percent == true: wait for
     * auth, timeout on matched (3 times). if matching
     * timeouted, trigger BFB. BFB should end up in auth again!
     * if all retries exceeded, set ac_use_5percent = false &&
     * BFB to WaitingForAuth If auth is received &&
     * !ac_enforce_hlc_with_eim, check if matching was started
     * -> go through BFB sequence to restart in nominal PWM mode
     * BFB goes to B1 when finished. set ac_use_5percent to
     * false. matched was already done -> go through  B2 X1
     * B2(nominal) to restart in nominal mode BX1B goes to B1
     * when finished. set ac_use_5percent to false. neither ->
     * keep 5% on, continue to B1 If 3 retries on trying to
     * match, restart with B2 X1 B2(nominal)
     *
     * dc_mode: use_5percent always true. wait for auth, keep 5%
     * on, continue to B1. No retries for DC.
     *
     * todo: execute DLINK commands etc
     * */
}

void Charger::updateDisplay() {
    char str[50];
#if YETI_RELEASE == 1
    ControlMode m = getControlMode();
    display->clearDisplay();

    sprintf(str, " %s %-5.5s RC %.1fmA", (oddeven ? "*" : " "),
            control_pilot.currentStateToString(), getResidualCurrent());
    display->printText(str, 0, 0, Display::LEFT);

    if (m == ControlMode::Low)
        sprintf(str, "PWM %.1f%% RL %s  ",
                control_pilot.getPwmDutyCycle() * 100.,
                (control_pilot.isPowerOn() ? "on" : "off"));
    else
        sprintf(str, "MaxC %.1fA RL %s  ", getMaxCurrent(),
                (control_pilot.isPowerOn() ? "on" : "off"));
    display->printText(str, 0, 1, Display::LEFT);

    if (m == ControlMode::Low) {
        sprintf(str, "CP %.1fV/%.1fV %s", control_pilot.getCPHi(),
                control_pilot.getCPLo(),
                (control_pilot.getThreePhasesConfirmed() ? "3ph" : "1ph"));
        display->printText(str, 0, 3, Display::LEFT);
        sprintf(str, "VS %.1fV/%.1fV", control_pilot.getSupply12V(),
                control_pilot.getSupplyN12V());
        display->printText(str, 0, 4, Display::LEFT);
    } else {
        evseStateToString(str, currentState);
        display->printText(str, 5, 4, Display::LEFT);
    }

    PowerMeterData pmd = getPowerMeterData();
    sprintf(str, "%2.1fkW %3.1fkWh",
            (pmd.watt[0] + pmd.watt[1] + pmd.watt[2]) / 1000.,
            pmd.totalWattHr / 1000.);
    display->printText(str, 0, 5, Display::LEFT);

    if (getControlMode() == ControlMode::None) {
        display->printText("[IEC61851]", 0, 7, Display::LEFT);
    } else if (getControlMode() == ControlMode::Low) {
        display->printText("[LO-CTRL]", 0, 7, Display::LEFT);
    } else {
        display->printText("[HI-CTRL]", 0, 7, Display::LEFT);
    }
    if (simulation) {
        display->printText("[SIM]", 13, 7, Display::LEFT);
    }

    display->updateDisplay();
    HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (GPIO_PinState)oddeven);
    oddeven = !oddeven;
#endif
}

float Charger::ampereToDutyCycle(float ampere) {
    float dc = 0;

    lastPwmUpdate = HAL_GetTick();

    // calculate max current
    if (ampere < 5.9) {
        // Invalid argument, switch to error
        dc = 1.0;
    } else if (ampere <= 6.1) {
        dc = 0.1;
    } else if (ampere < 52.5) {
        if (ampere > 51.)
            ampere = 51; // Weird gap in norm: 51A .. 52.5A has no defined PWM.
        dc = ampere / 0.6 / 100.;
    } else if (ampere <= 80) {
        dc = ((ampere / 2.5) + 64) / 100.;
    } else if (ampere <= 80.1) {
        dc = 0.97;
    } else {
        // Invalid argument, switch to error
        dc = 1.0;
    }

    return dc;
}

bool Charger::setMaxCurrent(float c) {
    OsMutexLockGuard lock(configMutex);
    if (c > 5.9 && c <= 80 && c <= hardCurrentLimit) {
        // FIXME: limit to cable limit (PP reading) if that is smaller!
        maxCurrent = c;
        return true;
    }
    return false;
}

void Charger::setHardCurrentLimit(float ampere) {
    OsMutexLockGuard lock(configMutex);
    hardCurrentLimit = ampere;
}

void Charger::setThreePhases(bool n) {
    OsMutexLockGuard lock(configMutex);
    control_pilot.setThreePhases(n);
}

// pause if currently charging, else do nothing.
bool Charger::pauseCharging() {
    if (controlMode == ControlMode::High) {
        OsMutexLockGuard lock(stateMutex);
        if (currentState == EvseState::Charging) {
            currentState = EvseState::ChargingPausedEVSE;
            return true;
        }
    }
    return false;
}

bool Charger::resumeCharging() {
    if (controlMode == ControlMode::High) {
        OsMutexLockGuard lock(stateMutex);
        if (currentState == EvseState::ChargingPausedEVSE) {
            currentState = EvseState::Charging;
            return true;
        }
    }
    return false;
}

bool Charger::switchThreePhasesWhileCharging(bool n) {
    return control_pilot.switchThreePhasesWhileCharging(n);
}

void Charger::setHasVentilation(bool v) {
    OsMutexLockGuard lock(configMutex);
    control_pilot.setHasVentilation(v);
}

void Charger::setCountryCode(const char *iso3166_alpha2_code) {
    control_pilot.setCountryCode(iso3166_alpha2_code);
}

void Charger::setControlMode(ControlMode c) {
    OsMutexLockGuard lock(configMutex);
    controlMode = c;
}

ControlMode Charger::getControlMode() {
    OsMutexLockGuard lock(configMutex);
    return controlMode;
}

Charger::EvseState Charger::getCurrentState() {
    OsMutexLockGuard lock(stateMutex);
    return currentState;
}

void Charger::Authorize(bool a, const char *userid) {
    OsMutexLockGuard lock(configMutex);
    authorized = a;
    // FIXME: do sth useful with userid
#if 0
if (a) {
    // copy to local user id
} else {
    // clear local userid
}
#endif
}

bool Charger::getAuthorization() {
    OsMutexLockGuard lock(configMutex);
    return authorized;
}

Charger::ErrorState Charger::getErrorState() {
    OsMutexLockGuard lock(stateMutex);
    return errorState;
}

bool Charger::getVentilatedChargingActive() {
    OsMutexLockGuard lock(stateMutex);
    return control_pilot.getVentilatedChargingActive();
}

bool Charger::isPowerOn() { return control_pilot.isPowerOn(); }

bool Charger::getThreePhases() {
    OsMutexLockGuard lock(configMutex);
    return control_pilot.getThreePhases();
}

bool Charger::getThreePhasesConfirmed() {
    OsMutexLockGuard lock(configMutex);
    return control_pilot.getThreePhasesConfirmed();
}

bool Charger::getHasVentilation() {
    OsMutexLockGuard lock(configMutex);
    return control_pilot.getHasVentilation();
}

Charger::DebugVars Charger::getDebugVars() {
    DebugVars d;
    /*
        float s_CPVoltage;
        bool s_errorE, s_diodeFail;
    */
    osMutexWait(stateMutex, osWaitForever);
    d.evsePwmRunning = control_pilot.getPwmRunning();
    d.simplifiedMode = control_pilot.getEvSimplifiedMode();
    d.currentState = control_pilot.getCurrentState();
    d.pwmDutyCycle = control_pilot.getPwmDutyCycle();
    d.rcdReclosingAllowed = control_pilot.getRcdReclosingAllowed();
    d.evseCPHi = control_pilot.getCPHi();
    d.evseCPLo = control_pilot.getCPLo();
    d.supply12V = control_pilot.getSupply12V();
    d.supplyN12V = control_pilot.getSupplyN12V();
    d.rcdEnabled = control_pilot.getRcdEnabled();
    osMutexRelease(stateMutex);

    osMutexWait(configMutex, osWaitForever);
    d.cpuTemperature = (float)power_meter.getTEMP(ADE7978::L1) *
                           ADE7978::TEMPERATURE_CELSIUS_SCALE +
                       ADE7978::TEMPERATURE_CELSIUS_OFFSET;

    d.maxCurrentCable = 0.;
    d.simulation = simulation;
    /*    s_CPVoltage = sim_CPVoltage;
        s_errorE = sim_errorE;
        s_diodeFail = sim_diodeFail;
        */
    osMutexRelease(configMutex);

    return d;
}

void Charger::rcdEnable() { control_pilot.rcdEnable(); }

void Charger::rcdDisable() { control_pilot.rcdDisable(); }

void Charger::keepAliveFromHi() {
    uint32_t t = HAL_GetTick();
    // if (t-lastKeepAliveFromHi < 1000) show some timeout on display
    // etc.
    lastKeepAliveFromHi = t;
}

Display *Charger::getDisplay() { return display; }

// Simulator interface
void Charger::enableSimulation(bool e) {
    // do nothing if not changed
    bool current;
    osMutexWait(configMutex, osWaitForever);
    current = simulation;
    osMutexRelease(configMutex);

    if (e == current)
        return;

    // are we switching off? reset some stuff...
    if (!e) {
        osMutexWait(stateMutex, osWaitForever);
        currentState = EvseState::Idle;
        osMutexRelease(stateMutex);
    }

    osMutexWait(stateMutex, osWaitForever);
    control_pilot.enableSimulation(e);
    osMutexRelease(stateMutex);

    osMutexWait(configMutex, osWaitForever);
    simulation = e;
    osMutexRelease(configMutex);
}

bool Charger::simulationEnabled() {
    OsMutexLockGuard lock(configMutex);
    return simulation;
}

void Charger::setSimulationData(float cpVoltage, float ppResistor,
                                float currents[4], float voltages[3],
                                float freq[3], float rcdCurrent, bool diodeFail,
                                bool errorE) {
    osMutexWait(configMutex, osWaitForever);

    sim_CPVoltage = cpVoltage;
    sim_ppResistor = ppResistor; // FIXME: no effect so far as PP
                                 // reading is not implemented
    memcpy(sim_currents, currents, 4 * sizeof(float));
    memcpy(sim_voltages, voltages, 3 * sizeof(float));
    memcpy(sim_freq, freq, 3 * sizeof(float));
    sim_rcdCurrent = rcdCurrent;

    sim_diodeFail = diodeFail;
    sim_errorE = errorE;

    osMutexRelease(configMutex);

    osMutexWait(stateMutex, osWaitForever);
    control_pilot.setSimulationData(cpVoltage, ppResistor, diodeFail, errorE,
                                    rcdCurrent);
    osMutexRelease(stateMutex);
}

void Charger::updatePowerMeter() {
    PowerMeterData d;

    if (simulation) {
        uint32_t currentTick = HAL_GetTick();
        float deltaT = (currentTick - sim_powermeter_lastTick) / 1000.;
        sim_powermeter_lastTick = currentTick;

        // Fill entirely with data
        d.vrms[ADE7978::L1] = sim_voltages[0];
        d.vrms[ADE7978::L2] = sim_voltages[1];
        d.vrms[ADE7978::L3] = sim_voltages[2];
        d.irms[ADE7978::L1] = sim_currents[0];
        d.irms[ADE7978::L2] = sim_currents[1];
        d.irms[ADE7978::L3] = sim_currents[2];
        d.irms[ADE7978::N] = sim_currents[3];

        sim_wattHr[0] +=
            d.vrms[ADE7978::L1] * d.irms[ADE7978::L1] * deltaT / 3600.;
        sim_wattHr[1] +=
            d.vrms[ADE7978::L2] * d.irms[ADE7978::L2] * deltaT / 3600.;
        sim_wattHr[2] +=
            d.vrms[ADE7978::L3] * d.irms[ADE7978::L3] * deltaT / 3600.;
        d.totalWattHr = sim_wattHr[0] + sim_wattHr[1] + sim_wattHr[2];

        d.wattHr[ADE7978::L1] = sim_wattHr[0];
        d.wattHr[ADE7978::L2] = sim_wattHr[1];
        d.wattHr[ADE7978::L3] = sim_wattHr[2];

        d.temp[ADE7978::L1] = (float)power_meter.getTEMP(ADE7978::L1) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::L2] = (float)power_meter.getTEMP(ADE7978::L2) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::L3] = (float)power_meter.getTEMP(ADE7978::L3) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::N] = (float)power_meter.getTEMP(ADE7978::N) *
                                 ADE7978::TEMPERATURE_CELSIUS_SCALE +
                             ADE7978::TEMPERATURE_CELSIUS_OFFSET;

        d.watt[ADE7978::L1] = d.vrms[ADE7978::L1] * d.irms[ADE7978::L1];
        d.watt[ADE7978::L2] = d.vrms[ADE7978::L2] * d.irms[ADE7978::L2];
        d.watt[ADE7978::L3] = d.vrms[ADE7978::L3] * d.irms[ADE7978::L3];

        d.freq[ADE7978::L1] = sim_freq[0];
        d.freq[ADE7978::L2] = sim_freq[1];
        d.freq[ADE7978::L3] = sim_freq[2];
        d.phaseSeqError = false;

    } else {
        d.vrms[ADE7978::L1] =
            (double)power_meter.getVRMS(ADE7978::L1) * ADE7978::VOLTAGE_UNIT;
        d.vrms[ADE7978::L2] =
            (double)power_meter.getVRMS(ADE7978::L2) * ADE7978::VOLTAGE_UNIT;
        d.vrms[ADE7978::L3] =
            (double)power_meter.getVRMS(ADE7978::L3) * ADE7978::VOLTAGE_UNIT;
        d.irms[ADE7978::L1] =
            (double)power_meter.getIRMS(ADE7978::L1) * ADE7978::CURRENT_UNIT;
        d.irms[ADE7978::L2] =
            (double)power_meter.getIRMS(ADE7978::L2) * ADE7978::CURRENT_UNIT;
        d.irms[ADE7978::L3] =
            (double)power_meter.getIRMS(ADE7978::L3) * ADE7978::CURRENT_UNIT;
        d.irms[ADE7978::N] =
            (double)power_meter.getIRMS(ADE7978::N) * ADE7978::CURRENT_UNIT;
        d.wattHr[ADE7978::L1] = (double)power_meter.getWATTHR(ADE7978::L1) *
                                ADE7978::DEFAULT_ENERGY_UNIT;
        d.wattHr[ADE7978::L2] = (double)power_meter.getWATTHR(ADE7978::L2) *
                                ADE7978::DEFAULT_ENERGY_UNIT;
        d.wattHr[ADE7978::L3] = (double)power_meter.getWATTHR(ADE7978::L3) *
                                ADE7978::DEFAULT_ENERGY_UNIT;
        d.totalWattHr =
            power_meter.getTotalWATTHR() * ADE7978::DEFAULT_ENERGY_UNIT;
#ifdef CHARGER_CPP_ENABLE_PRINTF
        // printf ("v1 %f i1 %f whr %f\n", d.vrms[0], d.irms[0],
        // d.totalWattHr);
#endif
        d.temp[ADE7978::L1] = (float)power_meter.getTEMP(ADE7978::L1) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::L2] = (float)power_meter.getTEMP(ADE7978::L2) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::L3] = (float)power_meter.getTEMP(ADE7978::L3) *
                                  ADE7978::TEMPERATURE_CELSIUS_SCALE +
                              ADE7978::TEMPERATURE_CELSIUS_OFFSET;
        d.temp[ADE7978::N] = (float)power_meter.getTEMP(ADE7978::N) *
                                 ADE7978::TEMPERATURE_CELSIUS_SCALE +
                             ADE7978::TEMPERATURE_CELSIUS_OFFSET;
#ifdef CHARGER_CPP_ENABLE_PRINTF
        // printf ("T1r %X T1 %f T2 %f T3 %f TN %f\n",
        // power_meter.getTEMP(ADE7978::L1), d.temp[0], d.temp[1],
        // d.temp[2], d.temp[3]);
#endif
        d.watt[ADE7978::L1] = (float)power_meter.getWATT(ADE7978::L1) *
                              ADE7978::DEFAULT_POWER_UNIT;
        d.watt[ADE7978::L2] = (float)power_meter.getWATT(ADE7978::L2) *
                              ADE7978::DEFAULT_POWER_UNIT;
        d.watt[ADE7978::L3] = (float)power_meter.getWATT(ADE7978::L3) *
                              ADE7978::DEFAULT_POWER_UNIT;
        float t =
            ((float)power_meter.getPERIOD(ADE7978::L1) * ADE7978::PERIOD_UNIT);
        d.freq[ADE7978::L1] = ((t > 1. / 70. && t < 1. / 40.) ? 1. / t : 0);
        t = ((float)power_meter.getPERIOD(ADE7978::L2) * ADE7978::PERIOD_UNIT);
        d.freq[ADE7978::L2] = ((t > 1. / 70. && t < 1. / 40.) ? 1. / t : 0);
        t = ((float)power_meter.getPERIOD(ADE7978::L3) * ADE7978::PERIOD_UNIT);
        d.freq[ADE7978::L3] = ((t > 1. / 70. && t < 1. / 40.) ? 1. / t : 0);
#ifdef CHARGER_CPP_ENABLE_PRINTF
        // printf ("f1 %f f2 %f f3 %f/%f\n", d.freq[0], d.freq[1],
        // d.freq[2], t);
#endif
        d.phaseSeqError = (power_meter.getSTATUS1() & (1 << 19) ? true : false);
#ifdef CHARGER_CPP_ENABLE_PRINTF
        // printf ("phaseSeqError %i\n", d.phaseSeqError);
#endif
    }

    osMutexWait(configMutex, osWaitForever);
    powerMeterData = d;
    osMutexRelease(configMutex);
}

void Charger::checkOverCurrent() {
    float softCurrentLimit = getMaxCurrent() * 1.1;

    // Hard current limit can be checked in any control mode and will emit a
    // over current event.
    if (powerMeterData.irms[ADE7978::L1] > hardCurrentLimit ||
        powerMeterData.irms[ADE7978::L2] > hardCurrentLimit ||
        powerMeterData.irms[ADE7978::L3] > hardCurrentLimit) {
        // printf ("HHHHHARD OVER CURRENT %f %f %f %f\n", hardCurrentLimit,
        // powerMeterData.irms[ADE7978::L1], powerMeterData.irms[ADE7978::L2],
        // powerMeterData.irms[ADE7978::L3]);
        control_pilot.setOverCurrent(true, hardOverCurrentTimeout);
    }

    // Soft current limit cannot be checked here in low control mode,
    // needs to be in charger implementation in everest then.
    else if (controlMode != ControlMode::Low &&
             (powerMeterData.irms[ADE7978::L1] > softCurrentLimit ||
              powerMeterData.irms[ADE7978::L2] > softCurrentLimit ||
              powerMeterData.irms[ADE7978::L3] > softCurrentLimit)) {
        // printf("SSSOFT OVER CURRENT %f %f %f %f\n", softCurrentLimit,
        //       powerMeterData.irms[ADE7978::L1],
        //       powerMeterData.irms[ADE7978::L2],
        //       powerMeterData.irms[ADE7978::L3]);
        control_pilot.setOverCurrent(true, softOverCurrentTimeout);
    }

    else
        control_pilot.setOverCurrent(false, 0);
}

bool Charger::disable() {
    OsMutexLockGuard lock(stateMutex);
    currentState = EvseState::Disabled;
    return true;
}

bool Charger::enable() {
    OsMutexLockGuard lock(stateMutex);
    if (currentState == EvseState::Disabled) {
        currentState = EvseState::Idle;
        return true;
    }
    return false;
}

bool Charger::restart() {
    if (controlMode == ControlMode::High) {
        OsMutexLockGuard lock(stateMutex);

        if (currentState == EvseState::Finished) {
            currentState = EvseState::Idle;
            return true;
        }
    }
    return false;
}

void Charger::evseStateToString(char *str, EvseState s) {
    switch (s) {
    case EvseState::Disabled:
        sprintf(str, "Disabled");
        break;
    case EvseState::Idle:
        sprintf(str, "Idle");
        break;
    case EvseState::WaitingForAuthentication:
        sprintf(str, "WaitAuth");
        break;
    case EvseState::Charging:
        sprintf(str, "Charging");
        break;
    case EvseState::ChargingPausedEV:
        sprintf(str, "Car Paused");
        break;
    case EvseState::ChargingPausedEVSE:
        sprintf(str, "EVSE Paused");
        break;
    case EvseState::Finished:
        sprintf(str, "Finished");
        break;
    case EvseState::Error:
        sprintf(str, "Error");
        break;
    case EvseState::Faulted:
        sprintf(str, "Faulted");
        break;
    }
}

float Charger::getResidualCurrent() {
    return control_pilot.getResidualCurrent();
}

float Charger::getMaxCurrent() {
    OsMutexLockGuard lock(configMutex);
    return maxCurrent;
}

Charger::PowerMeterData Charger::getPowerMeterData() {
    OsMutexLockGuard lock(configMutex);
    return powerMeterData;
}

// Proxy interface for control_mode == 2 (direct low level control)
void Charger::pwmOn(float dc) {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.pwmOn(dc);
    }
}

void Charger::pwmOff() {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.pwmOff();
    }
}


void Charger::replug(unsigned int t) {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.replug(t);
    }
}
void Charger::pwmF() {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.pwmF();
    }
}

void Charger::allowPowerOn(bool p) {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.allowPowerOn(p);
    }
}

void Charger::forceUnlock() {
    if (controlMode == ControlMode::Low) {
        OsMutexLockGuard lock(stateMutex);
        control_pilot.forceUnlock();
    }
}

void Charger::setRemoteControl(RemoteControl *p) { remote_control = p; }
