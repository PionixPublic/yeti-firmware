/*
 * PowerSwitch.cpp
 *
 *  Created on: 26.02.2021
 *      Author: cornelius
 */

#include "PowerSwitch.h"

PowerSwitch::PowerSwitch(TIM_HandleTypeDef *_pwmTimer, Gpio &_l1mirror,
                         Gpio &_l2l3mirror) :
    pwmTimer(_pwmTimer), l1mirror(_l1mirror), l2l3mirror(_l2l3mirror) {
    HAL_TIM_Base_Start_IT(pwmTimer);
    HAL_TIM_PWM_Start_IT(pwmTimer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(pwmTimer, TIM_CHANNEL_2);
    // Initialize to known state (off)
    switchOff();
    // printf("Powerswitch initialized: %i\n", relaisHealthy);
}

PowerSwitch::~PowerSwitch() { switchOff(); }

bool PowerSwitch::isOn() { return relaisOn; }

bool PowerSwitch::switchOnSinglePhase() {
    if (relaisHealthy) {
        setPWML1(100);
        setPWML2L3(0);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnSinglePhase 1: On");
#endif
        osDelay(relaisDelay);
        relaisOn = true;
        setPWML1(relaisHoldingPercent);
        setPWML2L3(0);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnSinglePhase 2: PWM mode");
#endif
        if (!l1mirror.read() && l2l3mirror.read())
            relaisHealthy = true;
        else
            relaisHealthy = false;
    }
    return relaisHealthy;
}

bool PowerSwitch::switchOnThreePhase() {
    if (relaisHealthy) {
        setPWML1(100);
        setPWML2L3(100);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnThreePhase 1: On");
#endif
        osDelay(relaisDelay);
        relaisOn = true;
        setPWML1(relaisHoldingPercent);
        setPWML2L3(relaisHoldingPercent);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnThreePhase 2: PWM mode");
#endif
        if (!l1mirror.read() && !l2l3mirror.read())
            relaisHealthy = true;
        else
            relaisHealthy = false;
    }
    return relaisHealthy;
}

bool PowerSwitch::switchOff() {
    setPWML1(0);
    setPWML2L3(0);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnThreePhase 1: Off");
#endif
    osDelay(relaisDelay);
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("switchOnThreePhase 2: Off after delay");
#endif
    relaisOn = false;
    if (l1mirror.read() && l2l3mirror.read())
        relaisHealthy = true;
    else
        relaisHealthy = false;
    return relaisHealthy;
}

void PowerSwitch::setPWML1(uint32_t dc) {
    uint16_t counterTicks = dc * 4800 / 100;
    pwmTimer->Instance->CCR1 = counterTicks;
}

void PowerSwitch::setPWML2L3(uint32_t dc) {
    uint16_t counterTicks = dc * 4800 / 100;
    pwmTimer->Instance->CCR2 = counterTicks;
}

bool PowerSwitch::executeSelfTest() {
    bool success = true;
    printf("   Relais self test...\n");
    if (switchOnSinglePhase()) {
        printf("OK PowerSwitch: SinglePhase on\n");
    } else {
        printf("FAIL PowerSwitch: SinglePhase on\n");
        success = false;
    }
    osDelay(100);

    if (switchOff()) {
        printf("OK PowerSwitch: SinglePhase off\n");
    } else {
        printf("FAIL PowerSwitch: SinglePhase off\n");
        success = false;
    }
    osDelay(100);

    if (switchOnThreePhase()) {
        printf("OK PowerSwitch: ThreePhase on\n");
    } else {
        printf("FAIL PowerSwitch: ThreePhase on\n");
        success = false;
    }
    osDelay(100);

    if (switchOff()) {
        printf("OK PowerSwitch: ThreePhase off\n");
    } else {
        printf("FAIL PowerSwitch: ThreePhase off\n");
        success = false;
    }
    return success;
}

void PowerSwitch::emergencySwitchOff() {
   /* relaisHealthy = false;
    setPWML1(0);
    setPWML2L3(0);
    relaisOn = false;
    printf("+++++++++++++++++++++++ EMERGENCY SWITCHOFF ++++++++++++++++++++++ "
           "\n");
           */
}

void PowerSwitch::resetEmergencySwitchOff() {
    // NOTE In the following countries automatic reclosing of protection means
    // is not allowed: DK, UK, FR, CH.
    relaisHealthy = true;
    setPWML1(0);
    setPWML2L3(0);
    relaisOn = false;
}
