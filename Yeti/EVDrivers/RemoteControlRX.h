/*
 * RemoteControl.h
 *
 *  Created on: 31.03.2021
 *      Author: cornelius
 */

#ifndef SRC_EVDRIVERS_REMOTECONTROLRX_H_
#define SRC_EVDRIVERS_REMOTECONTROLRX_H_

#include "MgmtLink.h"
#include "Task.h"
#include "OsMutexLockGuard.h"
#include "ControlPilot.h"
#include "RemoteControlTX.h"

// C Style callback for CMSIS RTOS Timer callback
extern "C" void RemoteControlTimerCallback(void *obj);

class RemoteControlRX: public Task {
public:
	RemoteControlRX(ControlPilot &_control_pilot, RemoteControlTX &_tx,
			MgmtLink &_link);
	virtual ~RemoteControlRX();

private:
	virtual void main();

	ControlPilot &control_pilot;
	RemoteControlTX &tx;
	MgmtLink &link;

	void startTimer(uint32_t msecs);
	void resetTimer();
	bool timerElapsed();
	uint32_t timer_countdown;
	uint32_t timer_tick;

};

#endif // SRC_EVDRIVERS_REMOTECONTROLRX_H_
