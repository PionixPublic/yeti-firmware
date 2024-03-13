/*
 * ControlPilot.h
 *
 *  Created on: 12.03.2024
 *  Author: cornelius
 *
 *  IEC 61851-1 compliant Control Pilot
 */

#ifndef SRC_EVDRIVERS_CONTROLPILOT_H_
#define SRC_EVDRIVERS_CONTROLPILOT_H_

#include "ControlPilot_HAL.h"
#include "PowerSwitch.h"
#include "ADE7978.h"
#include "Rcd.h"
#include "cmsis_os.h"
#include "main.h"
#include <queue>
#include "Task.h"
#include "RemoteControlTX.h"
#include "DataTypes.h"

class ControlPilot: public Task {
public:
	ControlPilot(ControlPilot_HAL &_control_pilot_hal,
			RemoteControlTX &_remote_tx, PowerSwitch &_power_switch, Rcd &_rcd,
			ADE7978 &_power_meter);
	virtual ~ControlPilot();

	void pwm_on(float dc);
	void pwm_off();
	void pwm_F();
	void replug(unsigned int t);

	// Allow to power on relais. The ControlPilot class decides when to actually
	// switch on and off. Switiching off is always allowed independent of this
	// setting. The flag will be reset e.g. if car unplugs or if pwmOff()/pwmF()
	// is called
	void allow_power_on(bool p);

	void connector_lock(bool lock);

	void rcd_enable();
	void rcd_disable();

	void set_over_current(bool o, uint32_t timeout);

	void set_three_phases(bool n);

private:

	// References to external objects we use here
	ControlPilot_HAL &control_pilot_hal;
	RemoteControlTX &remote_tx;
	PowerSwitch &power_switch;
	Rcd &rcd;
	ADE7978 &power_meter;

	virtual void main();

	void enable();
	void disable();

	void run_state_machine();

	InternalCPState current_state { InternalCPState::Disabled };
	InternalCPState last_state { InternalCPState::Disabled };

	bool current_relais_state { false };
	bool last_relais_state { false };

	bool tesla_last_pwm_running { false };
	uint32_t teslaPwmTimer { 0 };
	bool teslaPwmTimerStarted { false };

	bool read_from_car(InternalCPState *cp);

	bool power_on(bool threePhase);
	bool power_off();
	void check_lock();

	void check_over_current();

	bool overcurrent { false };
	uint32_t oc_timeout_tick { 0 };
	uint32_t oc_timeout { 7000 };

	bool is_voltage_in_range(float voltage, float center);

	void start_timer(uint32_t msecs);
	bool timer_elapsed();
	uint32_t timer_countdown { 0 };
	uint32_t timer_tick { 0 };

	float cp_hi { 0. };
	float cp_lo { 0. };

	bool use_three_phase { true };
	bool use_three_phase_confirmed { true };
	bool rcd_reclosing_allowed { true };

	bool replugging_in_progress { false };
	bool replugging_in_progress_F { false };
	bool replugging_in_progress_HI { false };
	bool replugging_start { false };
	int replugging_timer_F { 0 };
	int replugging_timer_HI { 0 };

	uint32_t lock_sw_off_Tick { 0 };
	float pwm_duty_cycle { 0. };

	bool pwm_running { false };
	bool last_pwm_running { false };
	bool power_on_allowed { false };

	PpState last_pp_state { PpState::PpState_STATE_NC };
	PpState current_pp_state { PpState::PpState_STATE_NC };

	// This mutex locks all state type members
	osMutexId_t state_mutex;
	const osMutexAttr_t state_mutex_attributes = { .name = "state_mutex" };

	ErrorFlags error_flags;
};

#endif // SRC_EVDRIVERS_CONTROLPILOT_H_
