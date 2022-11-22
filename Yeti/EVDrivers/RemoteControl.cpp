/*
 * RemoteControl.cpp
 *
 *  Created on: 31.03.2021
 *      Author: cornelius
 *
 * FIXME: the software timer callback waits on locks inside of charger object.
 * This means it blocks the timer thread in freeRTOS, which is not the best idea
 * as it causes jitter on other sw timer callbacks. For now we don't use them
 * anywhere else though.
 */

#include "RemoteControl.h"
#include "ADE7978.h"
#include "Charger.h"
#include "FirmwareUpdater.h"
#include "version_autogen.h"

extern int reset_cause_int;

RemoteControl::RemoteControl(Charger &_charger, MgmtLink &_link) :
    Task("RControl", 256 * 4), charger(_charger), link(_link) {
    txTimerArg = (uint32_t *)this;
    txTimer = osTimerNew(RemoteControlTimerCallback, osTimerPeriodic,
                         txTimerArg, NULL);
    txCnt = 0;
    sendMutex = osMutexNew(&sendMutex_attributes);
    osTimerStart(txTimer, 250);
}

RemoteControl::~RemoteControl() { osTimerStop(txTimer); }

/*
 * Main RX packet handling thread. Periodic TX is done in timer callback.
 * */
void RemoteControl::main() {
    HiToLo msg_in;

    // Send some dummy packets to ensure COBS resync
    sendKeepAlive();
    sendKeepAlive();
    sendKeepAlive();

    // Announce that we are done resetting
    reset_done();

    while (1) {
        if (link.read(&msg_in, 1000)) {
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
            printf("RemoteControl recvd %i \n", msg_in.which_payload);
#endif
            switch (msg_in.which_payload) {
            case HiToLo_firmware_update_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received FirmwareUpdate with: romBoot %i\n",
                       msg_in.payload.firmware_update.invoke_rom_bootloader);
#endif
                if (msg_in.payload.firmware_update.invoke_rom_bootloader) {
                    charger.suspend();
                    char str[50];
                    Display *display = charger.getDisplay();
                    display->clearDisplay();
                    sprintf(str, "ROM Bootloader");
                    display->printText(str, 0, 0, Display::LEFT);
                    sprintf(str, "Firmware update...");
                    display->printText(str, 0, 2, Display::LEFT);
                    display->updateDisplay();
                    osDelay(500);
                    restartInBootLoaderMode();
                }
                break;
            case HiToLo_enable_simulation_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received EnableSimulation with: %i\n",
                       msg_in.payload.enable_simulation.s);
#endif
                charger.enableSimulation(msg_in.payload.enable_simulation.s);
                break;
            case HiToLo_simulation_data_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received Simulation data.\n");
#endif
                {
                    float c[4];
                    float v[3];
                    float f[3];
                    c[0] = msg_in.payload.simulation_data.currentL1;
                    c[1] = msg_in.payload.simulation_data.currentL2;
                    c[2] = msg_in.payload.simulation_data.currentL3;
                    c[3] = msg_in.payload.simulation_data.currentN;

                    v[0] = msg_in.payload.simulation_data.voltageL1;
                    v[1] = msg_in.payload.simulation_data.voltageL2;
                    v[2] = msg_in.payload.simulation_data.voltageL3;

                    f[0] = msg_in.payload.simulation_data.freqL1;
                    f[1] = msg_in.payload.simulation_data.freqL2;
                    f[2] = msg_in.payload.simulation_data.freqL3;

                    charger.setSimulationData(
                        msg_in.payload.simulation_data.cp_voltage,
                        msg_in.payload.simulation_data.pp_resistor, c, v, f,
                        msg_in.payload.simulation_data.rcd_current,
                        msg_in.payload.simulation_data.diode_fail,
                        msg_in.payload.simulation_data.error_e);
                }
                break;
            case HiToLo_set_max_current_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received SetMaxCurrent with: %f ampere\n",
                       msg_in.payload.set_max_current.ampere);
#endif
                charger.setMaxCurrent(msg_in.payload.set_max_current.ampere);
                sendStateUpdate();
                break;
            case HiToLo_set_three_phases_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received SetThreePhases %i\n",
                       msg_in.payload.set_three_phases.n);
#endif
                charger.setThreePhases(msg_in.payload.set_three_phases.n);
                break;
            case HiToLo_set_has_ventilation_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received set_has_ventilation %i\n",
                       msg_in.payload.set_has_ventilation.v);
#endif
                charger.setHasVentilation(msg_in.payload.set_has_ventilation.v);
                break;
            case HiToLo_set_country_code_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received  set_country_code %s\n",
                       msg_in.payload.set_country_code.iso3166_alpha2_code);
#endif
                charger.setCountryCode(
                    msg_in.payload.set_country_code.iso3166_alpha2_code);
                break;
            case HiToLo_set_control_mode_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received control_mode %i\n",
                       msg_in.payload.set_control_mode.control_mode);
#endif
                charger.setControlMode(controlModeFromProtobuf(
                    msg_in.payload.set_control_mode.control_mode));
                break;
            case HiToLo_enable_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received enable\n");
#endif
                charger.enable();
                sendStateUpdate();
                break;
            case HiToLo_disable_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received disable\n");
#endif
                charger.disable();
                sendStateUpdate();
                break;
            case HiToLo_set_auth_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received set_auth %s\n",
                       msg_in.payload.set_auth.userid);
#endif
                charger.Authorize(true, msg_in.payload.set_auth.userid);
                sendStateUpdate();
                break;
            case HiToLo_switch_three_phases_while_charging_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received switch_three_phases_while_charging %i\n",
                       msg_in.payload.switch_three_phases_while_charging.n);
#endif
                charger.switchThreePhasesWhileCharging(
                    msg_in.payload.switch_three_phases_while_charging.n);
                sendStateUpdate();
                break;
            case HiToLo_pause_charging_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received pause_charging\n");
#endif
                charger.pauseCharging();
                sendStateUpdate();
                break;
            case HiToLo_resume_charging_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received resume_charging\n");
#endif
                charger.resumeCharging();
                sendStateUpdate();
                break;
            case HiToLo_enable_rcd_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received enable_rcd %i\n", msg_in.payload.enable_rcd.e);
#endif
                if (msg_in.payload.enable_rcd.e)
                    charger.rcdEnable();
                else
                    charger.rcdDisable();
                break;
                sendStateUpdate();
            case HiToLo_keep_alive_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received keep_alive %u\n",
                       (unsigned int)msg_in.payload.keep_alive.time_stamp);
#endif
                charger.keepAliveFromHi();
                break;
            case HiToLo_restart_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received restart\n");
#endif
                charger.restart();
                sendStateUpdate();
                break;
            case HiToLo_force_unlock_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received force_unlock\n");
#endif
                charger.forceUnlock();
                break;
            case HiToLo_allow_power_on_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received allow_poweron %i\n",
                       (int)msg_in.payload.allow_power_on.p);
#endif
                charger.allowPowerOn(msg_in.payload.allow_power_on.p);
                break;
            case HiToLo_set_pwm_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received set_pwm %i %f\n",
                       (int)msg_in.payload.set_pwm.mode,
                       msg_in.payload.set_pwm.duty_cycle);
#endif
                switch (msg_in.payload.set_pwm.mode) {
                case 0:
                    charger.pwmOff();
                    break;
                case 1:
                    charger.pwmOn(msg_in.payload.set_pwm.duty_cycle);
                    break;
                case 2:
                    charger.pwmF();
                    break;
                default:
                    break;
                }
                break;
            case HiToLo_reset_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received reset\n");
#endif
                reset_flags = 0;
                NVIC_SystemReset();
                break;
            case HiToLo_replug_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received replug %i\n", (int)msg_in.payload.replug.time);
#endif
                charger.replug(msg_in.payload.replug.time);
                break;
            }
        }
    }
}

void RemoteControl::sendPowerMeter() {
    // OsMutexLockGuard lock(sendMutex);
    Charger::PowerMeterData d = charger.getPowerMeterData();
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_power_meter_tag;
    msg_out.payload.power_meter.time_stamp = HAL_GetTick();
    msg_out.payload.power_meter.vrmsL1 = d.vrms[ADE7978::L1];
    msg_out.payload.power_meter.vrmsL2 = d.vrms[ADE7978::L2];
    msg_out.payload.power_meter.vrmsL3 = d.vrms[ADE7978::L3];
    msg_out.payload.power_meter.irmsL1 = d.irms[ADE7978::L1];
    msg_out.payload.power_meter.irmsL2 = d.irms[ADE7978::L2];
    msg_out.payload.power_meter.irmsL3 = d.irms[ADE7978::L3];
    msg_out.payload.power_meter.irmsN = d.irms[ADE7978::N];
    msg_out.payload.power_meter.wattHrL1 = d.wattHr[ADE7978::L1];
    msg_out.payload.power_meter.wattHrL2 = d.wattHr[ADE7978::L2];
    msg_out.payload.power_meter.wattHrL3 = d.wattHr[ADE7978::L3];
    msg_out.payload.power_meter.totalWattHr = d.totalWattHr;
    msg_out.payload.power_meter.tempL1 = d.temp[ADE7978::L1];
    msg_out.payload.power_meter.tempL2 = d.temp[ADE7978::L2];
    msg_out.payload.power_meter.tempL3 = d.temp[ADE7978::L3];
    msg_out.payload.power_meter.tempN = d.temp[ADE7978::N];
    msg_out.payload.power_meter.wattL1 = d.watt[ADE7978::L1];
    msg_out.payload.power_meter.wattL2 = d.watt[ADE7978::L2];
    msg_out.payload.power_meter.wattL3 = d.watt[ADE7978::L3];
    msg_out.payload.power_meter.freqL1 = d.freq[ADE7978::L1];
    msg_out.payload.power_meter.freqL2 = d.freq[ADE7978::L2];
    msg_out.payload.power_meter.freqL3 = d.freq[ADE7978::L3];
    msg_out.payload.power_meter.phaseSeqError = d.phaseSeqError;
    link.write(&msg_out);
}

void RemoteControl::sendKeepAlive() {
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_keep_alive_tag;
    msg_out.payload.keep_alive.time_stamp = HAL_GetTick();
    msg_out.payload.keep_alive.hw_type = 0;
    msg_out.payload.keep_alive.hw_revision = 0;
    msg_out.payload.keep_alive.protocol_version_major = 0;
    msg_out.payload.keep_alive.protocol_version_minor = 1;

    msg_out.payload.keep_alive.hwcap_max_current = hard_limit;
    msg_out.payload.keep_alive.hwcap_min_current = 6;
    msg_out.payload.keep_alive.hwcap_max_phase_count = 3;
    msg_out.payload.keep_alive.hwcap_min_phase_count = 1;
    msg_out.payload.keep_alive.supports_changing_phases_during_charging = true;

    strncpy(msg_out.payload.keep_alive.sw_version_string, VERSION_STRING, 50);
    msg_out.payload.keep_alive.sw_version_string[50] = 0;
    link.write(&msg_out);
}

void RemoteControl::sendStateUpdate() {
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_state_update_tag;
    msg_out.payload.state_update.time_stamp = HAL_GetTick();

    switch (charger.getCurrentState()) {
    case Charger::EvseState::Disabled:
        msg_out.payload.state_update.state = StateUpdate_State_DISABLED;
        break;
    case Charger::EvseState::Idle:
        msg_out.payload.state_update.state = StateUpdate_State_IDLE;
        break;
    case Charger::EvseState::WaitingForAuthentication:
        msg_out.payload.state_update.state =
            StateUpdate_State_WAITING_FOR_AUTHENTICATION;
        break;
    case Charger::EvseState::Charging:
        msg_out.payload.state_update.state = StateUpdate_State_CHARGING;
        msg_out.payload.state_update.which_state_flags =
            StateUpdate_charging_flags_tag;
        msg_out.payload.state_update.state_flags.charging_flags.ventilation =
            charger.getVentilatedChargingActive();
        break;
    case Charger::EvseState::ChargingPausedEV:
        msg_out.payload.state_update.state =
            StateUpdate_State_CHARGING_PAUSED_EV;
        break;
    case Charger::EvseState::ChargingPausedEVSE:
        msg_out.payload.state_update.state =
            StateUpdate_State_CHARGING_PAUSED_EVSE;
        break;
    case Charger::EvseState::Error:
        msg_out.payload.state_update.state = StateUpdate_State_ERROR;
        msg_out.payload.state_update.which_state_flags =
            StateUpdate_error_type_tag;
        switch (charger.getErrorState()) {
        case Charger::ErrorState::Error_E:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_E;
            break;
        case Charger::ErrorState::Error_F:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_F;
            break;
        case Charger::ErrorState::Error_DF:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_DF;
            break;
        case Charger::ErrorState::Error_Relais:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_RELAIS;
            break;
        case Charger::ErrorState::Error_VentilationNotAvailable:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_VENTILATION_NOT_AVAILABLE;
            break;
        case Charger::ErrorState::Error_RCD:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_RCD;
            break;
        case Charger::ErrorState::Error_OverCurrent:
            msg_out.payload.state_update.state_flags.error_type.type =
                ErrorFlags_ErrorType_ERROR_OVER_CURRENT;
            break;
        }
        break;
    case Charger::EvseState::Finished:
        msg_out.payload.state_update.state =
            StateUpdate_State_CHARGING_FINSIHED;
        break;
    case Charger::EvseState::Faulted:
        msg_out.payload.state_update.state = StateUpdate_State_FAULTED;
        break;
    }

    link.write(&msg_out);
}

void RemoteControl::sendSimulationFeedback() {
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_simulation_feedback_tag;

    Charger::DebugVars d = charger.getDebugVars();

    msg_out.payload.simulation_feedback.evse_pwm_running = d.evsePwmRunning;
    msg_out.payload.simulation_feedback.relais_on =
        (charger.isPowerOn() ? (charger.getThreePhasesConfirmed() ? 3 : 1) : 0);
    msg_out.payload.simulation_feedback.pwmDutyCycle = d.pwmDutyCycle;
    msg_out.payload.simulation_feedback.evse_pwm_voltage_hi = d.evseCPHi;
    msg_out.payload.simulation_feedback.evse_pwm_voltage_lo = d.evseCPLo;

    link.write(&msg_out);
}

void RemoteControl::sendDebugUpdate() {
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_debug_update_tag;
    msg_out.payload.debug_update.time_stamp = HAL_GetTick();

    Charger::DebugVars d = charger.getDebugVars();

    msg_out.payload.debug_update.evse_pwm_voltage_hi = d.evseCPHi;
    msg_out.payload.debug_update.evse_pwm_voltage_lo = d.evseCPLo;
    msg_out.payload.debug_update.supply_voltage_12V = d.supply12V;
    msg_out.payload.debug_update.supply_voltage_N12V = d.supplyN12V;
    msg_out.payload.debug_update.simulation = d.simulation;

#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
    printf("12V: %f, -12V: %f PWMhi %f, PWMLo %f\n", d.supply12V, d.supplyN12V,
           d.evseCPHi, d.evseCPLo);
#endif

    switch (d.currentState) {
    case ControlPilot::CPState::Disabled:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_DISABLED;
        break;
    case ControlPilot::CPState::A:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_A;
        break;
    case ControlPilot::CPState::B:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_B;
        break;
    case ControlPilot::CPState::C:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_C;
        break;
    case ControlPilot::CPState::D:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_D;
        break;

    case ControlPilot::CPState::E:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_E;
        break;
    case ControlPilot::CPState::F:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_F;
        break;
    case ControlPilot::CPState::DF:
        msg_out.payload.debug_update.lowlevel_state =
            DebugUpdate_LoLevelState_DF;
        break;
    }
    msg_out.payload.debug_update.evse_pwm_running =
        d.evsePwmRunning; // charger.getEvsePwmRunning();
    msg_out.payload.debug_update.ev_simplified_mode =
        d.simplifiedMode; // charger.getSimplifiedMode();
    msg_out.payload.debug_update.has_ventilation = charger.getHasVentilation();
    msg_out.payload.debug_update.ventilated_charging_active =
        charger.getVentilatedChargingActive();
    msg_out.payload.debug_update.rcd_reclosing_allowed =
        d.rcdReclosingAllowed; // charger.getRcdReclosingAllowed();
    msg_out.payload.debug_update.control_mode =
        controlModeToProtobuf(charger.getControlMode());
    msg_out.payload.debug_update.authorized = charger.getAuthorization();
    msg_out.payload.debug_update.cpu_temperature =
        d.cpuTemperature; // charger.getCpuTemperature();
    msg_out.payload.debug_update.rcd_enabled =
        d.rcdEnabled; // charger.rcd.getEnabled();
    msg_out.payload.debug_update.max_current_cable =
        d.maxCurrentCable; // charger.getMaxCurrentCable();
    msg_out.payload.debug_update.max_current =
        charger.getMaxCurrent(); // charger.getMaxCurrentCable();
    msg_out.payload.debug_update.rcd_current =
        charger.getResidualCurrent(); // charger.getMaxCurrentCable();

    msg_out.payload.debug_update.watchdog_reset_count = reset_cause_int;

    link.write(&msg_out);
}

InterfaceControlMode RemoteControl::controlModeToProtobuf(ControlMode c) {
    switch (c) {
    case ControlMode::None:
        return InterfaceControlMode::InterfaceControlMode_NONE;
        break;
    case ControlMode::High:
        return InterfaceControlMode::InterfaceControlMode_HIGH;
        break;
    case ControlMode::Low:
        return InterfaceControlMode::InterfaceControlMode_LOW;
        break;
    }
    return InterfaceControlMode::InterfaceControlMode_NONE;
}

ControlMode RemoteControl::controlModeFromProtobuf(InterfaceControlMode c) {
    switch (c) {
    case InterfaceControlMode::InterfaceControlMode_NONE:
        return ControlMode::None;
        break;
    case InterfaceControlMode::InterfaceControlMode_HIGH:
        return ControlMode::High;
        break;
    case InterfaceControlMode::InterfaceControlMode_LOW:
        return ControlMode::Low;
        break;
    }
    return ControlMode::None;
}

void RemoteControl::event(ControlPilot::Event e) {
    Event_InterfaceEvent ie;
    switch (e) {
    case ControlPilot::Event::CarPluggedIn:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_CAR_PLUGGED_IN;
        break;
    case ControlPilot::Event::CarRequestedPower:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_CAR_REQUESTED_POWER;
        break;
    case ControlPilot::Event::PowerOn:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_POWER_ON;
        break;
    case ControlPilot::Event::PowerOff:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_POWER_OFF;
        break;
    case ControlPilot::Event::CarRequestedStopPower:
        ie =
            Event_InterfaceEvent::Event_InterfaceEvent_CAR_REQUESTED_STOP_POWER;
        break;
    case ControlPilot::Event::CarUnplugged:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_CAR_UNPLUGGED;
        break;
    case ControlPilot::Event::Error_E:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ERROR_E;
        break;
    case ControlPilot::Event::Error_DF:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ERROR_DF;
        break;
    case ControlPilot::Event::Error_RCD:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ERROR_RCD;
        break;
    case ControlPilot::Event::Error_Relais:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ERROR_RELAIS;
        break;
    case ControlPilot::Event::Error_VentilationNotAvailable:
        ie = Event_InterfaceEvent::
            Event_InterfaceEvent_ERROR_VENTILATION_NOT_AVAILABLE;
        break;
    case ControlPilot::Event::Error_OverCurrent:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ERROR_OVER_CURRENT;
        break;
    case ControlPilot::Event::EnterBCD:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_ENTER_BCD;
        break;
    case ControlPilot::Event::LeaveBCD:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_LEAVE_BCD;
        break;
    case ControlPilot::Event::PermanentFault:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_PERMANENT_FAULT;
        break;
    case ControlPilot::Event::EvseReplugStarted:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_EVSE_REPLUG_STARTED;
        break;
    case ControlPilot::Event::EvseReplugFinished:
        ie = Event_InterfaceEvent::Event_InterfaceEvent_EVSE_REPLUG_FINISHED;
        break;
    }
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_event_tag;

    msg_out.payload.event.type = ie;

    link.write(&msg_out);
}

void RemoteControl::reset_done() {
    // OsMutexLockGuard lock(sendMutex);
    LoToHi msg_out = LoToHi_init_default;
    msg_out.which_payload = LoToHi_reset_done_tag;
    link.write(&msg_out);
}

void RemoteControl::timerCallback() {
    switch (txCnt++) {
    case 0:
        sendStateUpdate();
        break;
    case 1:
        sendKeepAlive();
        break;
    case 2:
        sendDebugUpdate();
        break;
    case 3:
        sendPowerMeter();
        if (!charger.simulationEnabled())
            txCnt = 0;
        break;
    case 4:
        sendSimulationFeedback();
        txCnt = 0;
        break;
    default:
        txCnt = 0;
        break;
    }
}

void RemoteControlTimerCallback(void *obj) {
    ((RemoteControl *)obj)->timerCallback();
}
