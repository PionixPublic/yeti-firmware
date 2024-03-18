/*
 * RemoteControl.cpp
 *
 *  Created on: 12.03.2024
 *      Author: cornelius
 *
 * FIXME: the software timer callback waits on locks inside of charger object.
 * This means it blocks the timer thread in freeRTOS, which is not the best idea
 * as it causes jitter on other sw timer callbacks. For now we don't use them
 * anywhere else though.
 */

#include <EVDrivers/RemoteControlRX.h>
#include "FirmwareUpdater.h"
#include "version_autogen.h"

extern int reset_cause_int;

RemoteControlRX::RemoteControlRX(ControlPilot &_control_pilot,
		RemoteControlTX &_tx, MgmtLink &_link) :
		Task("RControl", 256 * 4), control_pilot(_control_pilot), tx(_tx), link(
				_link) {
}

RemoteControlRX::~RemoteControlRX() {
}

/*
 * Main RX packet handling thread
 * */
void RemoteControlRX::main() {
	EverestToMcu msg_in;

	// Send some dummy packets to ensure COBS resync
	tx.send_keep_alive();
	tx.send_keep_alive();
	tx.send_keep_alive();

	// Announce that we are done resetting
	if (reset_cause_int == 2) {
		tx.send_reset_reason(ResetReason::ResetReason_WATCHDOG);
	} else {
		tx.send_reset_reason(ResetReason::ResetReason_USER);
	}

	startTimer(5000);

	while (1) {
		if (timerElapsed()) {
			//control_pilot.pwm_F();
			resetTimer();
		}

		if (link.read(&msg_in, 1000)) {
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
            printf("RemoteControl recvd %i \n", msg_in.which_payload);
#endif
			switch (msg_in.which_payload) {
			case EverestToMcu_firmware_update_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received FirmwareUpdate with: romBoot %i\n",
                       msg_in.payload.firmware_update.invoke_rom_bootloader);
#endif
				if (msg_in.payload.firmware_update.invoke_rom_bootloader) {
					control_pilot.suspend();
					restartInBootLoaderMode();
				}
				break;
			case EverestToMcu_keep_alive_tag:
				resetTimer();
				break;
			case EverestToMcu_connector_lock_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received connector_lock %i\n", (int)msg_in.payload.connector_lock);
#endif
				control_pilot.connector_lock(msg_in.payload.connector_lock);
				break;

			case EverestToMcu_pwm_duty_cycle_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
			                printf("Received pwm_duty_cycle %i\n",
			                       (int)msg_in.payload.pwm_duty_cycle);
			#endif
				if (msg_in.payload.pwm_duty_cycle == 0) {
					control_pilot.pwm_F();
				} else if (msg_in.payload.pwm_duty_cycle >= 10000) {
					control_pilot.pwm_off();
				} else {
					control_pilot.pwm_on(
							msg_in.payload.pwm_duty_cycle / 10000.);
				}
				break;

			case EverestToMcu_allow_power_on_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received allow_poweron %i\n",
                       (int)msg_in.payload.allow_power_on);
#endif
				control_pilot.allow_power_on(msg_in.payload.allow_power_on);
				break;

			case EverestToMcu_reset_tag:
#ifdef REMOTECONTROL_CPP_ENABLE_PRINTF
                printf("Received reset\n");
#endif
				reset_flags = 0;
				NVIC_SystemReset();
				break;
			}
		}
	}
}

void RemoteControlRX::startTimer(uint32_t msecs) {
	timer_countdown = msecs;
	timer_tick = HAL_GetTick();
}

void RemoteControlRX::resetTimer() {
	timer_tick = HAL_GetTick();
}

bool RemoteControlRX::timerElapsed() {
	if (timer_tick != 0 && HAL_GetTick() - timer_tick > timer_countdown)
		return true;
	else
		return false;
}
