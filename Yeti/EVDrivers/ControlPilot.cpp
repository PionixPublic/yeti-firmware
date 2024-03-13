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
		RemoteControlTX &_remote_tx, PowerSwitch &_power_switch, Rcd &_rcd, ADE7978& _power_meter) :
Task("ControlPilot", 2048 * 4), control_pilot_hal(_control_pilot_hal), remote_tx(
		_remote_tx), power_switch(_power_switch), rcd(_rcd), power_meter(_power_meter) {

	enable();
	pwm_F();

	rcd.enable(); // Default is RCD is enabled

	state_mutex = osMutexNew(&state_mutex_attributes);
}

ControlPilot::~ControlPilot() {
	pwm_F();
}

void ControlPilot::main() {

	// Enable CP output
	enable();

	int cnt = 0;

	while (true) {
		osDelay(50);

		if (cnt++ % 8 == 0) {
			Telemetry t;
			t.cp_voltage_hi = cp_hi;
			t.cp_voltage_lo = cp_lo;
			remote_tx.send_telemetry(t);

			// FIXME this needs to come from somewhere else
			PowerMeter p;
			remote_tx.send_power_meter(p);
		}

		osMutexWait(state_mutex, osWaitForever);

		// Run low level state machine update
		run_state_machine();

		// Send updates to EVerest if CP state changed
		if (last_state not_eq current_state) {
			last_state = current_state;
			remote_tx.send_cp_state(to_CpState(current_state));
		}

		// Send updates to EVerest if PP state changed
		if (last_pp_state not_eq current_pp_state) {
			last_pp_state = current_pp_state;
			remote_tx.send_pp_state(current_pp_state);
		}

		current_relais_state = power_switch.isOn();
		// Send updates to EVerest if relais state changed
		if (last_relais_state not_eq current_relais_state) {
			last_relais_state = current_relais_state;
			remote_tx.send_relais_state(current_relais_state);
		}

		osMutexRelease(state_mutex);
	}
}

void ControlPilot::run_state_machine() {

	// check if RCD fired in the meantime (actual emergency switch off happens
	// in interrupt)
	if (rcd.getRcdFired()) {
		rcd.reset(); // Note this does NOT reset the fault flag in the
					 // powerSwitch, just the flag that we do not keep on
					 // sending Error_RCD events!
		error_flags.rcd_triggered = true;
	}
	// check if we need to stop LOCK motor after one second
	check_lock();

	// check for (slow) over current situation:
	// e.g. car may not follow PWM current limit within N seconds
	check_over_current();

	// update currentState from Car reading if signal is stable
	if (read_from_car(&current_state)) {
		if (replugging_start) {
			replugging_start = false;
			replugging_in_progress = true;
			replugging_in_progress_F = true;
			replugging_in_progress_HI = false;
			pwm_F();
		}
		if (replugging_in_progress) {
			if (replugging_in_progress_F && (replugging_timer_F -= 50) <= 0) {
				disable(); // After F we disable (high impedance) for a while
				replugging_in_progress_F = false;
				replugging_in_progress_HI = true;
			}
			if (replugging_in_progress_HI && (replugging_timer_HI -= 50) <= 0) {
				enable();
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
		switch (current_state) {
		case InternalCPState::Disabled:
			// simply wait until someone enables us...
			power_on_allowed = false;
			error_flags.diode_fault = false;
			power_off();
			break;

		case InternalCPState::A:
			error_flags.diode_fault = false;
			use_three_phase_confirmed = use_three_phase;
			power_on_allowed = false;
			error_flags.rcd_triggered = false;
			pwm_off();
			power_off();
			power_switch.resetEmergencySwitchOff();
			rcd.reset();
			break;

		case InternalCPState::B:
			error_flags.diode_fault = false;
			power_off();
			break;

		case InternalCPState::D:
		case InternalCPState::C:
			error_flags.diode_fault = false;
			if (!pwm_running) { // C1
				// Table A.6 Sequence 10.2: EV does not stop drawing power
				// even if PWM stops. Stop within 7 seconds (E.g. Kona1!)
				// EVerest should stop after 6 seconds already
				if (last_pwm_running)
					start_timer(7000);
				if (timer_elapsed()) {
					// force power off under load
					power_off();
				}
			} else { // C2
				if (power_on_allowed) {
					// Table A.6: Sequence 4 EV ready to charge.
					// Must enable power within 3 seconds.
					power_on(use_three_phase_confirmed);
				}
			}
			break;

		case InternalCPState::E:
		case InternalCPState::F:
			error_flags.diode_fault = false;
			power_off();
			pwm_off();
			break;

		case InternalCPState::DF:
			error_flags.diode_fault = true;
			power_off();
			break;
		}

		last_pwm_running = pwm_running;
	}
}

void ControlPilot::pwm_off() {
	control_pilot_hal.setPWM(1.01);
	pwm_duty_cycle = 1.;
	pwm_running = false;
	power_on_allowed = false;
}

void ControlPilot::pwm_on(float dc) {
	control_pilot_hal.setPWM(dc);
	pwm_duty_cycle = dc;
	pwm_running = true;
}

void ControlPilot::replug(unsigned int t) {
	replugging_timer_F = t;
	replugging_timer_HI = t;
	replugging_start = true;
}

/*
 * Note that F can be exited by pwmOff or pwmOn only.
 * */
void ControlPilot::pwm_F() {
	// EVSE error - constant -12V signal on CP
	control_pilot_hal.setPWM(0.);
	pwm_duty_cycle = 0.;
	current_state = InternalCPState::F;
	pwm_running = false;
	power_on_allowed = false;
}

void ControlPilot::enable() {
	current_state = InternalCPState::A;
	pwm_off();
	control_pilot_hal.enableCP();
}

void ControlPilot::disable() {
	current_state = InternalCPState::Disabled;
	pwm_off();
	control_pilot_hal.disableCP();
}

// Translate ADC readings for lo and hi part of PWM to IEC61851 states.
// returns false if signal is unstable/invalid and cp argument was not
// updated.
bool ControlPilot::read_from_car(InternalCPState *cp) {
	bool cp_signal_valid = false;

	if (control_pilot_hal.readCPSignal()) {
		cp_lo = control_pilot_hal.getCPLo();
		cp_hi = control_pilot_hal.getCPHi();
		if (*cp == InternalCPState::Disabled)
			return true; // stay in Disabled independent of measurement
		else
			cp_signal_valid = true;
	}

	if (cp_signal_valid) {
		// CP-PE short or signal somehow gone. According to the norm both high and low voltage should be 0 (full short before diode).
		// Some test equipment implement that wrong however, shorting CP after the diode resulting in an undefined state (it would be E2).
		// This means we only check the hi part here.
		//if (isVoltageInRange(cpLo, 0.) && isVoltageInRange(cpHi, 0.))
		if (is_voltage_in_range(cp_hi, 0.)) {
			*cp = InternalCPState::E;
			return true;
		}
		// sth is wrong with negative signal
		if (pwm_running && !is_voltage_in_range(cp_lo, -12.)) {
			if (is_voltage_in_range(cp_hi + cp_lo, 0.)) {
				// Diode fault
				*cp = InternalCPState::DF;
			} else {
				return false;
			}
		} else if (is_voltage_in_range(cp_hi, 12.)) {
			// +12V State A IDLE (open circuit)
			*cp = InternalCPState::A;
		} else if (is_voltage_in_range(cp_hi, 9.)) {
			*cp = InternalCPState::B;
		} else if (is_voltage_in_range(cp_hi, 6.)) {
			*cp = InternalCPState::C;
		} else if (is_voltage_in_range(cp_hi, 3.)) {
			*cp = InternalCPState::D;
		} else if (is_voltage_in_range(cp_hi, -12.)) {
			*cp = InternalCPState::F;
		} else {
			return false;
		}
		return true;
	}
	return false;
}

// checks if voltage is within center+-interval
bool ControlPilot::is_voltage_in_range(float voltage, float center) {
	const float interval = 1.1;

	return ((voltage > center - interval) && (voltage < center + interval));
}

void ControlPilot::set_three_phases(bool n) {
	use_three_phase = n;
}

bool ControlPilot::power_on(bool threePhase) {
	bool success = true;
	if (!power_switch.isOn()) {
		rcd.deactivate();
		if (threePhase) {
			success = power_switch.switchOnThreePhase();
		} else {
			success = power_switch.switchOnSinglePhase();
		}
		rcd.activate();
		lock_sw_off_Tick = HAL_GetTick();
	}
	return success;
}

bool ControlPilot::power_off() {
	bool success = true;
	if (power_switch.isOn()) {
		// disable RCD
		rcd.deactivate();
		// actually switch off relais
		success = power_switch.switchOff();
		lock_sw_off_Tick = HAL_GetTick();
	}
	return success;
}

void ControlPilot::check_lock() {
	if (lock_sw_off_Tick != 0 && HAL_GetTick() - lock_sw_off_Tick > 1000) {
		control_pilot_hal.lockMotorOff();
		lock_sw_off_Tick = 0;
	}
}

void ControlPilot::set_over_current(bool o, uint32_t timeout) {
	oc_timeout = timeout;
	if (!overcurrent && o) {
		oc_timeout_tick = HAL_GetTick();
	}
	overcurrent = o;
}

void ControlPilot::check_over_current() {
	if (overcurrent && oc_timeout_tick != 0
			&& HAL_GetTick() - oc_timeout_tick > oc_timeout) {
		error_flags.over_current = true;
	} else {
		error_flags.over_current = false;
	}
}

void ControlPilot::rcd_enable() {
	rcd.enable();
}

void ControlPilot::rcd_disable() {
	rcd.disable();
}

void ControlPilot::start_timer(uint32_t msecs) {
	timer_countdown = msecs;
	timer_tick = HAL_GetTick();
}

bool ControlPilot::timer_elapsed() {
	if (timer_tick != 0 && HAL_GetTick() - timer_tick > timer_countdown)
		return true;
	else
		return false;
}

void ControlPilot::allow_power_on(bool p) {
	power_on_allowed = p;
}

void ControlPilot::connector_lock(bool lock) {
	if (lock) {
		control_pilot_hal.lockMotorLock();
		lock_sw_off_Tick = HAL_GetTick();
	} else {
		control_pilot_hal.lockMotorUnlock();
		lock_sw_off_Tick = HAL_GetTick();
	}
}
