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

#include <EVDrivers/RemoteControlTX.h>
#include "FirmwareUpdater.h"
#include "version_autogen.h"

extern int reset_cause_int;

RemoteControlTX::RemoteControlTX(MgmtLink &_link) :
		link(_link) {
}

RemoteControlTX::~RemoteControlTX() {
	osTimerStop(txTimer);
}

void RemoteControlTX::send_reset_reason(ResetReason r) {
	// OsMutexLockGuard lock(sendMutex);
	printf ("------------------------------- RESET TAG SENT ------------------");
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_reset_tag;
	msg_out.payload.reset = r;
	link.write(&msg_out);
}

void RemoteControlTX::send_cp_state(CpState cp_state) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_cp_state_tag;
	msg_out.payload.cp_state = cp_state;
	link.write(&msg_out);
}

void RemoteControlTX::send_relais_state(bool on) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_relais_state_tag;
	msg_out.payload.relais_state = on;
	link.write(&msg_out);
}

void RemoteControlTX::send_error_flags(ErrorFlags e) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_error_flags_tag;
	msg_out.payload.error_flags = e;
	link.write(&msg_out);
}

void RemoteControlTX::send_telemetry(Telemetry t) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_telemetry_tag;
	msg_out.payload.telemetry = t;
	link.write(&msg_out);
}

void RemoteControlTX::send_pp_state(PpState pp_state) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_cp_state_tag;
	msg_out.payload.pp_state = pp_state;
	link.write(&msg_out);
}

void RemoteControlTX::send_lock_state(LockState lock_state) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_lock_state_tag;
	msg_out.payload.lock_state = lock_state;
	link.write(&msg_out);
}

void RemoteControlTX::send_power_meter(PowerMeter p) {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_power_meter_tag;
	msg_out.payload.power_meter = p;
	link.write(&msg_out);
}

void RemoteControlTX::send_keep_alive() {
	// OsMutexLockGuard lock(sendMutex);
	McuToEverest msg_out = McuToEverest_init_default;
	msg_out.which_payload = McuToEverest_keep_alive_tag;
	msg_out.payload.keep_alive.time_stamp = HAL_GetTick();
	msg_out.payload.keep_alive.hw_type = 0;
	msg_out.payload.keep_alive.hw_revision = 0;
	msg_out.payload.keep_alive.protocol_version_major = 0;
	msg_out.payload.keep_alive.protocol_version_minor = 1;
	msg_out.payload.keep_alive.hwcap_min_current = 6;
	msg_out.payload.keep_alive.hwcap_max_current = 32;
	msg_out.payload.keep_alive.hwcap_min_phase_count = 1;
	msg_out.payload.keep_alive.hwcap_max_phase_count = 3;
	msg_out.payload.keep_alive.supports_changing_phases_during_charging = true;

	strncpy(msg_out.payload.keep_alive.sw_version_string, VERSION_STRING, 50);
	msg_out.payload.keep_alive.sw_version_string[50] = 0;
	link.write(&msg_out);
	printf ("Keep alive sent %i.\n", (int)msg_out.payload.keep_alive.time_stamp);
}
