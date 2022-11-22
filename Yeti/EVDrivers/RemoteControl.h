/*
 * RemoteControl.h
 *
 *  Created on: 31.03.2021
 *      Author: cornelius
 */

#ifndef SRC_EVDRIVERS_REMOTECONTROL_H_
#define SRC_EVDRIVERS_REMOTECONTROL_H_

#include "MgmtLink.h"
#include "Task.h"
#include "OsMutexLockGuard.h"
#include "ControlMode.h"
#include "ControlPilot.h"

// C Style callback for CMSIS RTOS Timer callback
extern "C" void RemoteControlTimerCallback(void *obj);
extern float hard_limit;

class Charger;

class RemoteControl : public Task {
public:
    RemoteControl(Charger &_charger, MgmtLink &_link);
    virtual ~RemoteControl();
    void timerCallback();

    void event(ControlPilot::Event e);
    void reset_done();

private:
    virtual void main();
    void sendStateUpdate();
    void sendKeepAlive();
    void sendDebugUpdate();
    void sendPowerMeter();
    void sendSimulationFeedback();

    ControlMode controlModeFromProtobuf(InterfaceControlMode c);
    InterfaceControlMode controlModeToProtobuf(ControlMode c);

    osMutexId_t sendMutex;
    const osMutexAttr_t sendMutex_attributes = {.name = "sendMutex"};

    Charger &charger;
    MgmtLink &link;
    osTimerId_t txTimer;
    uint32_t *txTimerArg;

    uint8_t txCnt;
};

#endif // SRC_EVDRIVERS_REMOTECONTROL_H_
