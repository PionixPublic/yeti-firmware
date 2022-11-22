/*
 * ControlPilot_HAL.h
 *
 *  Created on: 25.10.2021
 *  Author: cornelius
 *
 * IEC 61851-1 compliant Control Pilot state machine
 *
 * This class provides HAL abstraction for CP and PP signals:
 * 1) timer interrupt triggered ADC readings of PWM signal on CP
 * 2) PWM output on CP
 * 3) CP enable/disable (high impedance)
 * 4) PP reading (not implemented yet)
 * 5) Lock motor control
 * 6) supply voltage reading (uses the same ADC as CP signal sampling)
 *
 * TODO: Lock motor GPIO is currently hard coded
 */

#ifndef SRC_EVDRIVERS_CONTROLPILOT_HAL_H_
#define SRC_EVDRIVERS_CONTROLPILOT_HAL_H_

#include "cmsis_os.h"
#include "main.h"

#include "Adc.h"
#include "Gpio.h"
#include "InterruptBase.h"

class ControlPilot_HAL : private InterruptBase {
public:
    ControlPilot_HAL(TIM_HandleTypeDef *_pwmTimer, Adc &_adc, Gpio *_cpEnable);
    virtual ~ControlPilot_HAL();

    bool readCPSignal();
    float getCPHi();
    float getCPLo();
    float getSupply12V();
    float getSupplyN12V();

    void lockMotorLock();
    void lockMotorUnlock();
    void lockMotorOff();

    void setPWM(float dc);
    void disableCP();
    void enableCP();

    // Interrupt callback
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

private:
    // References to external objects we use here
    TIM_HandleTypeDef *pwmTimer;
    Adc &adc;

    float cpLo, cpHi;

    Gpio *cpEnable;
};

#endif // SRC_EVDRIVERS_CONTROLPILOT_HAL_H_
