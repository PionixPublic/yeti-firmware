/*
 * RemoteControl.h
 *
 *  Created on: 31.03.2021
 *      Author: cornelius
 */

#ifndef SRC_EVDRIVERS_REMOTECONTROLTX_H_
#define SRC_EVDRIVERS_REMOTECONTROLTX_H_

#include "MgmtLink.h"

class RemoteControlTX {
public:
	RemoteControlTX(MgmtLink &_link);
	virtual ~RemoteControlTX();

	void send_reset_reason(ResetReason r);
	void send_cp_state(CpState s);
	void send_relais_state(bool on);
	void send_error_flags(ErrorFlags f);
	void send_telemetry(Telemetry t);
	void send_pp_state(PpState p);
	void send_lock_state(LockState l);
	void send_power_meter(PowerMeter p);
	void send_keep_alive();

private:

	MgmtLink &link;

	osTimerId_t txTimer;
	uint32_t *txTimerArg;
	uint8_t txCnt;
};

#endif // SRC_EVDRIVERS_REMOTECONTROLTX_H_
