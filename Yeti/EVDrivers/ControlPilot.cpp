/*
 * ControlPilot.cpp
 *
 *  Created on: 15.10.2021
 *      Author: cornelius
 *
 */

#include "ControlPilot.h"

#include <math.h>
#include <string.h>

#include "EVConfig.h"

ControlPilot::ControlPilot(ControlPilot_HAL &_control_pilot_hal,
                           PowerSwitch &_powerSwitch, Rcd &_rcd) :
    control_pilot_hal(_control_pilot_hal),
    powerSwitch(_powerSwitch),
    rcd(_rcd) {

    pwmDutyCycle = 0.;
    useThreePhase = true;
    useThreePhaseConfirmed = true;
    hasVentilation = false;
    ocTimeout = 7000;

    overcurrent = false;
    ocTimeoutTick = 0;

    rcdReclosingAllowed = true;
    ventilatedChargingActive = false;

    disable();
    lastState = currentState;

    pwmRunning = false;
    lastPwmRunning = false;
    powerOnAllowed = false;

    timerCountdown = 0;
    timerTick = 0;

    currentState = CPState::Disabled;

    rcd.enable(); // Default is RCD is enabled

    simulation = false;
    sim_CPVoltage = 12.;
    sim_ppResistor = 220.;
    sim_diodeFail = false;
    sim_errorE = false;
    sim_rcdCurrent = 0.;

    replugging_in_progress = false;
    replugging_start = false;
}

ControlPilot::~ControlPilot() { pwmF(); }

std::queue<ControlPilot::Event> ControlPilot::runStateMachine() {

    std::queue<ControlPilot::Event> events;

    // check if RCD fired in the meantime (actual emergency switch off happens
    // in interrupt)
    if (rcd.getRcdFired()) {
        rcd.reset(); // Note this does NOT reset the fault flag in the
                     // powerSwitch, just the flag that we do not keep on
                     // sending Error_RCD events!
        events.push(Event::Error_RCD);
    }
    // check if we need to stop LOCK motor after one second
    checkLock();

    // check for (slow) over current situation:
    // e.g. car may not follow PWM current limit within N seconds
    checkOverCurrent(events);

    // update currentState from Car reading if signal is stable
#ifdef IGNORE_TESLA_SPECIAL_SEQUENCE
    if (teslaFilter_readFromCar(&currentState)) {
#else
    if (readFromCar(&currentState)) {
#endif
        if (replugging_start) {
            replugging_start = false;
            replugging_in_progress = true;
            replugging_in_progress_F = true;
            replugging_in_progress_HI = false;
            pwmF();
            events.push(Event::EvseReplugStarted);
        }
        if (replugging_in_progress) {
            if (replugging_in_progress_F && (replugging_timer_F -= 50) <= 0) {
                disable(); // After F we disable (high impedance) for a while
                replugging_in_progress_F = false;
                replugging_in_progress_HI = true;
            }
            if (replugging_in_progress_HI && (replugging_timer_HI -= 50) <= 0) {
                enable();
                events.push(Event::EvseReplugFinished);
                replugging_in_progress_HI = false;
                replugging_in_progress = false;
            }
            // we need to set F or high impedance, wait time t and then reset
            // replugging_in_progress and emit no events during that time also
            // disable power wenn car reacts etc. need to think about this.
            // but we need to follow normal state matrix internally.
            // basically after the time is up, we wait for state B. If that is
            // reached, we say it was done!
        }
        switch (currentState) {
        case CPState::Disabled:
            // simply wait until someone enables us...
            powerOnAllowed = false;
            powerOff(events);
            break;

        case CPState::A:
            useThreePhaseConfirmed = useThreePhase;
            pwmOff();
            evSimplifiedMode = false;
            ventilatedChargingActive = false;

            // Table A.6: Sequence 2.1 Unplug at state Bx (or any other
            // state) Table A.6: Sequence 2.2 Unplug at state Cx, Dx
            if (lastState != CPState::A && lastState != CPState::Disabled &&
                lastState != CPState::F) {
                events.push(Event::CarRequestedStopPower);
                powerOff(events);
                events.push(Event::CarUnplugged);

                // If car was unplugged, reset RCD flag.
                if (rcdReclosingAllowed) {
                    powerSwitch.resetEmergencySwitchOff();
                    rcd.reset();
                }
            }
            break;

        case CPState::B:
            // Table A.6: Sequence 7 EV stops charging
            // Table A.6: Sequence 8.2 EV supply equipment
            // responds to EV opens S2 (w/o PWM)

            if (lastState != CPState::A && lastState != CPState::B) {
                events.push(Event::CarRequestedStopPower);
                // Need to switch off according to Table A.6 Sequence 8.1
                // within
                powerOff(events);
            }

            ventilatedChargingActive = false;

            // Table A.6: Sequence 1.1 Plug-in
            if (lastState == CPState::A || lastState == CPState::Disabled) {
                events.push(Event::CarPluggedIn);
                evSimplifiedMode = false;
            }

            if (lastState == CPState::E || lastState == CPState::F) {
                events.push(Event::EnterBCD);
            }

            if (!pwmRunning) { // B1
            } else {           // B2
            }
            break;

        case CPState::C:
            // Table A.6: Sequence 1.2 Plug-in
            if (lastState == CPState::A) {
                events.push(Event::CarPluggedIn);
                evSimplifiedMode = true;
            }
            if (lastState == CPState::B) {
                events.push(Event::CarRequestedPower);
            }
            if (lastState == CPState::E || lastState == CPState::F) {
                events.push(Event::EnterBCD);
            }

            if (!pwmRunning) { // C1
                // Table A.6 Sequence 10.2: EV does not stop drawing power
                // even if PWM stops. Stop within 6 seconds (E.g. Kona1!)
                if (lastPwmRunning)
                    startTimer(6000);
                if (timerElapsed()) {
                    // force power off under load
                    powerOff(events);
                }
            } else { // C2
                if (powerOnAllowed) {
                    // Table A.6: Sequence 4 EV ready to charge.
                    // Must enable power within 3 seconds.
                    powerOn(events, useThreePhaseConfirmed);

                    // Simulate Request power Event here for simplified mode
                    // to ensure that this mode behaves similar for higher
                    // layers. Note this does not work with 5% mode
                    // correctly, but simplified mode does not support HLC
                    // anyway.
                    if (!lastPwmRunning && evSimplifiedMode)
                        events.push(Event::CarRequestedPower);
                }
            }
            break;

        case CPState::D:
            // Table A.6: Sequence 1.2 Plug-in (w/ventilation)
            if (lastState == CPState::A) {
                events.push(Event::CarPluggedIn);
                events.push(Event::CarRequestedPower);
                evSimplifiedMode = true;
            }

            if (lastState == CPState::B) {
                events.push(Event::CarRequestedPower);
            }

            if (lastState == CPState::E || lastState == CPState::F) {
                events.push(Event::EnterBCD);
            }

            if (!pwmRunning) {
                // Table A.6 Sequence 10.2: EV does not stop drawing power
                // even if PWM stops. Stop within 6 seconds (E.g. Kona1!)
                if (lastPwmRunning)
                    startTimer(6000);
                if (timerElapsed()) {
                    // force power off under load
                    powerOff(events);
                }
            } else {
                if (powerOnAllowed && !powerSwitch.isOn()) {
                    // Table A.6: Sequence 4 EV ready to charge.
                    // Must enable power within 3 seconds.
                    if (!hasVentilation)
                        events.push(Event::Error_VentilationNotAvailable);
                    else
                        powerOn(events, useThreePhaseConfirmed);
                }
                if (lastState == CPState::C) {
                    // Car switches to ventilation while charging.
                    if (!hasVentilation)
                        events.push(Event::Error_VentilationNotAvailable);
                }
            }
            break;

        case CPState::E:
            if (lastState != currentState)
                events.push(Event::Error_E);
            if (lastState == CPState::B || lastState == CPState::C ||
                lastState == CPState::D) {
                events.push(Event::LeaveBCD);
            }
            powerOff(events);
            pwmOff();
            break;
        case CPState::F:
            powerOff(events);
            if (lastState == CPState::B || lastState == CPState::C ||
                lastState == CPState::D) {
                events.push(Event::LeaveBCD);
            }
            break;
        case CPState::DF:
            if (lastState != currentState)
                events.push(Event::Error_DF);
            if (lastState == CPState::B || lastState == CPState::C ||
                lastState == CPState::D) {
                events.push(Event::LeaveBCD);
            }
            powerOff(events);
            break;
        }

        lastState = currentState;
        lastPwmRunning = pwmRunning;
    }
    return events;
}

void ControlPilot::pwmOff() {
    control_pilot_hal.setPWM(1.01);
    pwmDutyCycle = 1.;
    pwmRunning = false;
    powerOnAllowed = false;
}

void ControlPilot::pwmOn(float dc) {
    control_pilot_hal.setPWM(dc);
    pwmDutyCycle = dc;
    pwmRunning = true;
}

void ControlPilot::replug(unsigned int t) {
    replugging_timer_F = t;
    replugging_timer_HI = t;
    replugging_start = true;
}

/*
 * Note that F can be exited by pwmOff or pwmOn only.
 * */
void ControlPilot::pwmF() {
    // EVSE error - constant -12V signal on CP
    control_pilot_hal.setPWM(0.);
    pwmDutyCycle = 0.;
    currentState = CPState::F;
    pwmRunning = false;
    powerOnAllowed = false;
}

void ControlPilot::enable() {
    currentState = CPState::A;
    pwmOff();
    control_pilot_hal.enableCP();
}

void ControlPilot::disable() {
    currentState = CPState::Disabled;
    pwmOff();
    control_pilot_hal.disableCP();
}

bool ControlPilot::teslaFilter_readFromCar(CPState *cp) {
    // Filter out weird tesla special sequence at the start of HLC session:
    // Tesla does 5 short transitions B->C-DF->C->B or similar when 5%
    // PWM starts. During the first 16 seconds after 5% PWM was enabled, state
    // B->C/D/DF transitions need to persist for at least 1.5 second.
    CPState new_cp = *cp;

    // // Was PWM enabled since the last run of this function?
    if (!tesla_last_pwm_running && pwmRunning) {
        // start timer
        teslaPwmTimer = HAL_GetTick();
        teslaPwmTimerStarted = true;
    }
    tesla_last_pwm_running = pwmRunning;

    // Are we within the first 16 seconds after 5% PWM was enabled?
    if (teslaPwmTimerStarted && pwmRunning &&
        (HAL_GetTick() - teslaPwmTimer < 16000) && pwmDutyCycle > 0.04 &&
        pwmDutyCycle < 0.06) {

        if (readFromCar(&new_cp)) {

            if (tesla_filter_last_cp != new_cp) {
                teslaTick = HAL_GetTick();
                teslaTickStarted = true;
            }
            tesla_filter_last_cp = new_cp;

            if (teslaTickStarted && (HAL_GetTick() - teslaTick < 1500)) {
                // Ignore transition if it does not last for 1500ms
                return false;
            } else {
                teslaTickStarted = false;
                *cp = new_cp;
                return true;
            }

        } else {
            // readFromCar could not read a stable state
            return false;
        }

    } else {
        teslaPwmTimerStarted = false;
        // Do nothing here
        return readFromCar(cp);
    }
}

// Translate ADC readings for lo and hi part of PWM to IEC61851 states.
// returns false if signal is unstable/invalid and cp argument was not
// updated.
bool ControlPilot::readFromCar(CPState *cp) {
    bool cp_signal_valid = false;

    if (simulation) {
        if (sim_errorE) {
            cpHi = cpLo = 0.;
        } else {
            cpHi = sim_CPVoltage;
            if (sim_diodeFail) {
                cpLo = -cpHi;
            } else {
                if (pwmRunning)
                    cpLo = -12.;
                else
                    cpLo = cpHi;
            }
        }
        cp_signal_valid = true;
    } else {
        if (control_pilot_hal.readCPSignal()) {
            cpLo = control_pilot_hal.getCPLo();
            cpHi = control_pilot_hal.getCPHi();
            if (*cp == CPState::Disabled)
                return true; // stay in Disabled independent of measurement
            else
                cp_signal_valid = true;
        }
    }

    if (cp_signal_valid) {
        // sth is wrong with negative signal
        if (pwmRunning && !isVoltageInRange(cpLo, -12.)) {
            // CP-PE short or signal somehow gone
            if (isVoltageInRange(cpLo, 0.) && isVoltageInRange(cpHi, 0.))
                *cp = CPState::E;
            // Diode fault
            else if (isVoltageInRange(cpHi + cpLo, 0.)) {
                *cp = CPState::DF;
            } else
                return false;
        } else if (isVoltageInRange(cpHi, 12.)) {
            // +12V State A IDLE (open circuit)
            *cp = CPState::A;
        } else if (isVoltageInRange(cpHi, 9.)) {
            *cp = CPState::B;
        } else if (isVoltageInRange(cpHi, 6.)) {
            *cp = CPState::C;
        } else if (isVoltageInRange(cpHi, 3.)) {
            *cp = CPState::D;
        } else if (isVoltageInRange(cpHi, -12.)) {
            *cp = CPState::F;
        } else {
            return false;
        }
        return true;
    }
    return false;
}

// checks if voltage is within center+-interval
bool ControlPilot::isVoltageInRange(float voltage, float center) {
    const float interval = 1.1;

    return ((voltage > center - interval) && (voltage < center + interval));
}

void ControlPilot::setThreePhases(bool n) { useThreePhase = n; }

// FIXME implement me on new HW
bool ControlPilot::switchThreePhasesWhileCharging(bool n) {
    return true;
    /*
    EvseState s;
    int timeoutMs = 10000;

    bool prevUseThreePhase;
    // get previous state
    osMutexWait(configMutex, osWaitForever);
    prevUseThreePhase = useThreePhase;
    osMutexRelease(configMutex);

    // we can set number of phases right away, state machine
    // will update number of phases on next switch on only.
    // XXX setThreePhases(n) this may lead to fire. only switch in A1;
    // request pause charging
    pauseCharging(); // FIXME: this needs to be pause + unplug with new HW
to
                     // disable signal completely.
    // e.g. Kona does not really stop charging if asked...
    // number of phases will update once charging resumes.
    while (true) {

        osMutexWait(stateMutex, osWaitForever);
        s = currentState;
        osMutexRelease(stateMutex);

        if (s == EvseState::B1)
            break;
        osDelay(500);
        timeoutMs -= 500;
        if (timeoutMs < 0) {
#ifdef CHARGER_CPP_ENABLE_PRINTF
            printf("switchThreePhasesWhileCharging(): timeout.\n");
#endif
            useThreePhase = prevUseThreePhase;
            return false;
        }
    }
#ifdef CHARGER_CPP_ENABLE_PRINTF
    printf("switchThreePhasesWhileCharging(): Paused charging. Now "
           "unplugging...\n");
#endif

    control_pilot_hal.disableCP();
    // wait for State machine to arrive in A1
    while (true) {

        osMutexWait(stateMutex, osWaitForever);
        s = currentState;
        osMutexRelease(stateMutex);

        if (s == EvseState::A1)
            break;
        osDelay(500);
        timeoutMs -= 500;
        if (timeoutMs < 0) {
#ifdef CHARGER_CPP_ENABLE_PRINTF
            printf("switchThreePhasesWhileCharging(): timeout.\n");
#endif
            useThreePhase = prevUseThreePhase;
            return false;
        }
    }
    control_pilot_hal.enableCP();
    return true;
    */
}

void ControlPilot::setHasVentilation(bool v) { hasVentilation = v; }

bool ControlPilot::powerOn(std::queue<ControlPilot::Event> &events,
                           bool threePhase) {
    bool success = true;
    if (!powerSwitch.isOn()) {
        rcd.deactivate();
        if (threePhase)
            success = powerSwitch.switchOnThreePhase();
        else
            success = powerSwitch.switchOnSinglePhase();
        if (success)
            events.push(Event::PowerOn);
        else
            events.push(Event::Error_Relais);
        rcd.activate();
        // lock connector here
        control_pilot_hal.lockMotorLock();
        lockSwOffTick = HAL_GetTick();
    }
    return success;
}

bool ControlPilot::powerOff(std::queue<ControlPilot::Event> &events) {
    bool success = true;
    if (powerSwitch.isOn()) {
        // disable RCD
        rcd.deactivate();
        // actually switch off relais
        success = powerSwitch.switchOff();
        if (success)
            events.push(Event::PowerOff);
        else
            events.push(Event::Error_Relais);
        // unlock connector lock
        control_pilot_hal.lockMotorUnlock();
        lockSwOffTick = HAL_GetTick();
    }
    return success;
}

void ControlPilot::checkLock() {
    if (lockSwOffTick != 0 && HAL_GetTick() - lockSwOffTick > 1000) {
        control_pilot_hal.lockMotorOff();
        lockSwOffTick = 0;
    }
}

void ControlPilot::setOverCurrent(bool o, uint32_t timeout) {
    ocTimeout = timeout;
    if (!overcurrent && o) {
        ocTimeoutTick = HAL_GetTick();
    }
    overcurrent = o;
}

void ControlPilot::checkOverCurrent(std::queue<ControlPilot::Event> &events) {
    if (overcurrent && ocTimeoutTick != 0 &&
        HAL_GetTick() - ocTimeoutTick > ocTimeout) {
        events.push(Event::Error_OverCurrent);
        ocTimeoutTick = HAL_GetTick();
    }
}

void ControlPilot::setCountryCode(const char *iso3166_alpha2_code) {
    rcdReclosingAllowed = true;
    if (strcmp(iso3166_alpha2_code, "CH") == 0 ||
        strcmp(iso3166_alpha2_code, "DK") == 0 ||
        strcmp(iso3166_alpha2_code, "GB") == 0 ||
        strcmp(iso3166_alpha2_code, "FR") == 0) {
        // Recosing of RCD is not allowed in these countries after a
        // failure.
        rcdReclosingAllowed = false;
    }
}

bool ControlPilot::getVentilatedChargingActive() {
    return ventilatedChargingActive;
}

bool ControlPilot::isPowerOn() { return powerSwitch.isOn(); }

bool ControlPilot::getThreePhases() { return useThreePhase; }

bool ControlPilot::getThreePhasesConfirmed() { return useThreePhaseConfirmed; }

float ControlPilot::getResidualCurrent() {
    if (simulation)
        return sim_rcdCurrent;
    else
        return rcd.getResidualCurrent();
}

bool ControlPilot::getHasVentilation() { return hasVentilation; }

bool ControlPilot::getPwmRunning() { return pwmRunning; }

bool ControlPilot::getEvSimplifiedMode() { return evSimplifiedMode; }

float ControlPilot::getPwmDutyCycle() { return pwmDutyCycle; }

bool ControlPilot::getRcdReclosingAllowed() { return rcdReclosingAllowed; }

float ControlPilot::getCPHi() { return cpHi; }
float ControlPilot::getCPLo() { return cpLo; }

void ControlPilot::rcdEnable() { rcd.enable(); }

void ControlPilot::rcdDisable() { rcd.disable(); }

bool ControlPilot::getRcdEnabled() { return rcd.getEnabled(); };

float ControlPilot::getSupply12V() { return control_pilot_hal.getSupply12V(); }
float ControlPilot::getSupplyN12V() {
    return control_pilot_hal.getSupplyN12V();
}

ControlPilot::CPState ControlPilot::getCurrentState() { return currentState; }

// Simulator interface
void ControlPilot::enableSimulation(bool e) {
    // do nothing if not changed

    if (e == simulation)
        return;

    // are we switching off? reset some stuff...
    if (!e) {
        rcd.reset();
        currentState = CPState::A;
    }
    simulation = e;
}

bool ControlPilot::simulationEnabled() { return simulation; }

void ControlPilot::setSimulationData(float _sim_CPVoltage,
                                     float _sim_ppResistor, bool _sim_diodeFail,
                                     bool _sim_errorE, float _sim_rcdCurrent) {
    sim_CPVoltage = _sim_CPVoltage;
    sim_ppResistor = _sim_ppResistor;
    sim_diodeFail = _sim_diodeFail;
    sim_errorE = _sim_errorE;
    sim_rcdCurrent = _sim_rcdCurrent;

    if (sim_rcdCurrent > 6.) {
        rcd.fire();
    }
}

void ControlPilot::startTimer(uint32_t msecs) {
    timerCountdown = msecs;
    timerTick = HAL_GetTick();
}

bool ControlPilot::timerElapsed() {
    if (timerTick != 0 && HAL_GetTick() - timerTick > timerCountdown)
        return true;
    else
        return false;
}

const char *ControlPilot::eventToString(ControlPilot::Event e) {
    switch (e) {
    case ControlPilot::Event::CarPluggedIn:
        return "CarPluggedIn";
        break;
    case ControlPilot::Event::CarRequestedPower:
        return "CarRequestedPower";
        break;
    case ControlPilot::Event::PowerOn:
        return "PowerOn";
        break;
    case ControlPilot::Event::PowerOff:
        return "PowerOff";
        break;
    case ControlPilot::Event::CarRequestedStopPower:
        return "CarRequestedStopPower";
        break;
    case ControlPilot::Event::CarUnplugged:
        return "CarUnplugged";
        break;
    case ControlPilot::Event::Error_E:
        return "Error_E";
        break;
    case ControlPilot::Event::Error_DF:
        return "Error_DF";
        break;
    case ControlPilot::Event::Error_Relais:
        return "Error_Relais";
        break;
    case ControlPilot::Event::Error_RCD:
        return "Error_RCD";
        break;
    case ControlPilot::Event::Error_VentilationNotAvailable:
        return "Error_VentilationNotAvailable";
        break;
    case ControlPilot::Event::Error_OverCurrent:
        return "Error_OverCurrent";
        break;
    case ControlPilot::Event::EnterBCD:
        return "EnterBCD";
        break;
    case ControlPilot::Event::LeaveBCD:
        return "LeaveBCD";
        break;
    case ControlPilot::Event::PermanentFault:
        return "PermanentFault";
        break;
    case ControlPilot::Event::EvseReplugStarted:
        return "EvseReplugStarted";
        break;
    case ControlPilot::Event::EvseReplugFinished:
        return "EvseReplugFinished";
        break;
    }
    return "";
}

const char *ControlPilot::currentStateToString() {
    switch (currentState) {
    case CPState::Disabled:
        return "DS";
        break;
    case CPState::A:
        if (pwmRunning)
            return "A2";
        else
            return "A1";
        break;
    case CPState::B:
        if (pwmRunning)
            return "B2";
        else
            return "B1";
        break;
    case CPState::C:
        if (pwmRunning)
            return "C2";
        else
            return "C1";
        break;
    case CPState::D:
        if (pwmRunning)
            return "D2";
        else
            return "D1";
        break;
    case CPState::E:
        return "E ";
        break;
    case CPState::F:
        return "F ";
        break;
    case CPState::DF:
        return "DF";
        break;
    }
    return "UN";
}

void ControlPilot::allowPowerOn(bool p) { powerOnAllowed = p; }

void ControlPilot::forceUnlock() {
    // unlock connector lock
    control_pilot_hal.lockMotorUnlock();
    lockSwOffTick = HAL_GetTick();
}
