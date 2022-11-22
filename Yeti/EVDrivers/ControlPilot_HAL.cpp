/*
 * ControlPilot_HAL.cpp
 *
 *  Created on: 15.10.2021
 *      Author: cornelius
 *
 */

#include "ControlPilot_HAL.h"

#include <math.h>
#include <string.h>

#include "EVConfig.h"

ControlPilot_HAL::ControlPilot_HAL(TIM_HandleTypeDef *_pwmTimer, Adc &_adc,
                                   Gpio *_cpEnable) :
    pwmTimer(_pwmTimer), adc(_adc), cpEnable(_cpEnable) {

    registerCallback(IntType::HAL_TIM_PWM_PulseFinishedCallback);

    HAL_TIM_Base_Start_IT(pwmTimer);
    HAL_TIM_OC_Start_IT(pwmTimer, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(pwmTimer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(pwmTimer, TIM_CHANNEL_1);

    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 5, 0);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);

    disableCP();
}

ControlPilot_HAL::~ControlPilot_HAL() {
    removeCallback(IntType::HAL_TIM_PWM_PulseFinishedCallback);
    setPWM(0.);
}

void ControlPilot_HAL::setPWM(float dc) {
    uint16_t counterTicks = dc * 48000;
    pwmTimer->Instance->CCR1 = counterTicks;
}

// reads ADC values for lo and hi part of PWM
// returns false if signal is unstable
bool ControlPilot_HAL::readCPSignal() {
    // note we do not sqrt() the std dev here
    const float maxStdDev = 0.01; // 0.1V std dev

    float cpLoArr[Adc::AVG];
    float cpHiArr[Adc::AVG];

    adc.getEvseCPLo(cpLoArr);
    adc.getEvseCPHi(cpHiArr);

    // do some statistics to see if signal is stable
    float avgLo = 0;
    float avgHi = 0;
    for (int i = 0; i < Adc::AVG; i++) {
        avgLo += cpLoArr[i];
        avgHi += cpHiArr[i];
    }
    avgLo /= Adc::AVG;
    avgHi /= Adc::AVG;

    float stdDevLo = 0;
    float stdDevHi = 0;
    for (int i = 0; i < Adc::AVG; i++) {
        stdDevLo += (cpLoArr[i] - avgLo) * (cpLoArr[i] - avgLo);
        stdDevHi += (cpHiArr[i] - avgHi) * (cpHiArr[i] - avgHi);
    }
    stdDevLo /= Adc::AVG;
    stdDevHi /= Adc::AVG;

    if (stdDevHi > maxStdDev || stdDevLo > maxStdDev)
        return false;

    cpLo = avgLo;
    cpHi = avgHi;

    return true;
}

// End of high pulse in PWM cycle / general CC match callback
// Used here for CC2/CC3 that triggers ADC readings at the right moments
void ControlPilot_HAL::HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef *htim) {
    if (htim->Instance == pwmTimer->Instance) {
        // printf ("PWM\n");
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            // HI sample
            adc.triggerEvseCPHiISR();
            // enable GPIO output to debug ADC timing on PWM signal with scope
            // HAL_GPIO_WritePin(CABLE_20A_GPIO_Port, CABLE_20A_Pin,
            // GPIO_PIN_SET); HAL_GPIO_WritePin(CABLE_20A_GPIO_Port,
            // CABLE_20A_Pin, GPIO_PIN_RESET);
        } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            // LO sample
            adc.triggerEvseCPLoISR();
            // HAL_GPIO_WritePin(CABLE_20A_GPIO_Port, CABLE_20A_Pin,
            // GPIO_PIN_SET); HAL_GPIO_WritePin(CABLE_20A_GPIO_Port,
            // CABLE_20A_Pin, GPIO_PIN_RESET);
        }
    }
}

void ControlPilot_HAL::enableCP() {
    if (cpEnable)
        cpEnable->set();
}

void ControlPilot_HAL::disableCP() {
    if (cpEnable)
        cpEnable->reset();
}

void ControlPilot_HAL::lockMotorUnlock() {
    HAL_GPIO_WritePin(LOCK_F_GPIO_Port, LOCK_F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LOCK_R_GPIO_Port, LOCK_R_Pin, GPIO_PIN_RESET);
}

void ControlPilot_HAL::lockMotorLock() {
    HAL_GPIO_WritePin(LOCK_F_GPIO_Port, LOCK_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOCK_R_GPIO_Port, LOCK_R_Pin, GPIO_PIN_SET);
}

void ControlPilot_HAL::lockMotorOff() {
    HAL_GPIO_WritePin(LOCK_F_GPIO_Port, LOCK_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOCK_R_GPIO_Port, LOCK_R_Pin, GPIO_PIN_RESET);
}

float ControlPilot_HAL::getCPHi() { return cpHi; }
float ControlPilot_HAL::getCPLo() { return cpLo; }

float ControlPilot_HAL::getSupply12V() { return adc.getSupply12V(); }
float ControlPilot_HAL::getSupplyN12V() { return adc.getSupplyN12V(); }
