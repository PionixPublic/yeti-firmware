/*
 * Rcd.cpp
 *
 *  Created on: 01.03.2021
 *      Author: cornelius
 */

#include "Rcd.h"

#include <stdio.h>

#include "main.h"

uint16_t rcd_dc_errorin_gpio_pin = 0;
GPIO_TypeDef *rcd_dc_errorin_gpio_port = 0;
uint16_t rcd_ac_errorin_gpio_pin = 0;
GPIO_TypeDef *rcd_ac_errorin_gpio_port = 0;

Rcd::Rcd(Gpio &_testout, Gpio &_errorin, Gpio &_pwmin,
         TIM_HandleTypeDef *_pwmTimer, PowerSwitch *_powerSwitch) :
    testout(_testout),
    errorin(_errorin),
    pwmin(_pwmin),
    pwmTimer(_pwmTimer),
    powerSwitch(_powerSwitch) {

    rcd_dc_errorin_gpio_pin = _errorin.getPin();
    rcd_dc_errorin_gpio_port = _errorin.getPort();

    // rcd_ac_errorin_gpio_pin = 0;
    // rcd_ac_errorin_gpio_port = 0;

    testout.set(); // Note logic is inverted. HIGH means no self test.

    residualCurrentPercentPerMa = 2; // -X921 UL version
    // residualCurrentPercentPerMa=3.33; // -X920 IEC version

    // 96MHz timer clock base, 8KHz PWM expected from sensor
    // FIXME: this clock base could be retreived via HAL_RCC_Get*Freq
    expectedPwmPeriod = 96000 / 8;

    isrPwmUpdateTimeout = 0;
    isrPwmUpdated = false;
    isrRcdTestModeError = false;
    isrRcdEnabled = false;
    isrRcdFired = false;
    isrRcdActive = false;

    HAL_TIM_Base_Start(pwmTimer);
    HAL_TIM_IC_Start_IT(pwmTimer, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(pwmTimer, TIM_CHANNEL_2);

    // Register interrupt callback for CC channels (PWM decoding)
    registerCallback(IntType::HAL_TIM_IC_CaptureCallback);
    // Callback for overflow of PWM decode timer: Used to detect if PWM is
    // missing
    registerCallback(IntType::HAL_TIM_PeriodElapsedCallback);
    registerCallback(IntType::HAL_GPIO_EXTI_Callback);
}

Rcd::~Rcd() {
    removeCallback(IntType::HAL_TIM_IC_CaptureCallback);
    removeCallback(IntType::HAL_TIM_PeriodElapsedCallback);
    removeCallback(IntType::HAL_GPIO_EXTI_Callback);
}

//
// The routine triggers interal self testing of RCD sensor and
// monitors the PWM output and ERROR out pins. The test busy waits
// until the self test sequence is over.
//
bool Rcd::executeSelfTest() {
#ifdef RCD_CPP_ENABLE_PRINTF
    printf("   RCD self test...\n");
    // printf ("  RCD error line %i", (errorin.read()?1:0));
#endif

    deactivate();

    testout.set();
    osDelay(100);
    testout.reset();
    osDelay(100);
    testout.set();
    int i = 1000;
    int countResCurrent = 5; // needs to be n times over limit
    float c;
#ifdef RCD_CPP_ENABLE_PRINTF
    printf("   RC value ");
#endif

    for (i = 0; i < 100; ++i) {
        osDelay(100);
        c = getResidualCurrent();
#ifdef RCD_CPP_ENABLE_PRINTF
        printf(" %0.1fmA", c);
#endif
        if (c > 10 && countResCurrent > 0)
            countResCurrent--;
        if (countResCurrent == 0 && c < 0.2 && !errorin.read())
            break;
    }
#ifdef RCD_CPP_ENABLE_PRINTF
    printf("\n");
#endif

    bool success = true;

    // Fail if either:
    // - not returning to about 0mA after self test within 10 seconds
    // - not enough residual current values over limit (10mA)/10 times
    // - did not trigger FAIL pin
    if (c > 0.2) {
#ifdef RCD_CPP_ENABLE_PRINTF
        printf("FAIL RCD self test: Residual current too high: %.1f mA\n", c);
#endif
        success = false;
    }
    if (countResCurrent > 0) {
#ifdef RCD_CPP_ENABLE_PRINTF
        printf("FAIL RCD self test: Residual current not high for at least 10 "
               "measurements\n");
#endif
        success = false;
    }
    if (!isrRcdTestModeError) {
#ifdef RCD_CPP_ENABLE_PRINTF
        printf("FAIL RCD self test: Interrupt not triggered\n");
#endif
        success = false;
    }
#ifdef RCD_CPP_ENABLE_PRINTF
    if (success)
        printf("OK RCD self test.\n");
#endif
    return success;
}

float Rcd::getResidualCurrent() {
    float dc;
    taskENTER_CRITICAL();
    dc = isrPwmDutyCyclePercent;
    taskEXIT_CRITICAL();
    return dc / residualCurrentPercentPerMa;
}

//
// Detect missing PWM signal and reset isrPwmDutyCyclePercent if needed.
// Timeout is no valid update for 10 PWM cycles.
//
void Rcd::HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == pwmTimer->Instance) {
        if (isrPwmUpdateTimeout == 0) {
            // 10 PWM cycles is timeout for PWM receiver
            isrPwmUpdateTimeout = 10;
            isrPwmDutyCyclePercent = 0.;
        }
        if (!isrPwmUpdated)
            isrPwmUpdateTimeout--;
        isrPwmUpdated = false;
    }
}

void Rcd::HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // Check for all interrupt flags that relate to my PWM timer
    if (htim->Instance == pwmTimer->Instance) {

        switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            isrLastRising = isrRising;
            isrRising = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
            isrPwmPeriod = (isrRising - isrLastRising);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            isrFalling = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
            if (isrPwmPeriod > (expectedPwmPeriod - 500) &&
                isrPwmPeriod < (expectedPwmPeriod + 500)) {
                isrPwmDutyCyclePercent =
                    ((uint16_t)(isrFalling - isrRising) * 100) /
                    (float)isrPwmPeriod;
                isrPwmUpdated = true;
            }
            break;
        default:
            break;
        }
    }
}

void Rcd::fire() {
    if (isrRcdEnabled && isrRcdActive) {
        // emergency switch off relais
#ifdef FINE_GRAIN_DEBUG_PRINTF
        printf("RCD INT FIRED\n");
#endif
        if (powerSwitch)
            powerSwitch->emergencySwitchOff();

        isrRcdFired = true;
        isrRcdActive = false;
    }
}

void Rcd::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == rcd_dc_errorin_gpio_pin) {
        isrRcdTestModeError = true;
        if (isrRcdEnabled && isrRcdActive) {
            // emergency switch off relais
            if (powerSwitch)
                powerSwitch->emergencySwitchOff();

            isrRcdFired = true;
            isrRcdActive = false;
        }
#ifdef RCD_CPP_ENABLE_PRINTF
        else
            printf("RCD Error interrupt ignored!\n");
#endif
    }
}

void Rcd::enable() {
    taskENTER_CRITICAL();
    isrRcdEnabled = true;
    taskEXIT_CRITICAL();
}
void Rcd::disable() {
    taskENTER_CRITICAL();
    isrRcdEnabled = false;
    taskEXIT_CRITICAL();
}

void Rcd::activate() {
    taskENTER_CRITICAL();
    isrRcdActive = true;
    isrRcdTestModeError = false;
    isrRcdFired = false;
    taskEXIT_CRITICAL();
}
void Rcd::deactivate() {
    taskENTER_CRITICAL();
    isrRcdActive = false;
    isrRcdTestModeError = false;
    isrRcdFired = false;
    taskEXIT_CRITICAL();
}

void Rcd::reset() {
	taskENTER_CRITICAL();
	isrRcdFired = false;
	taskEXIT_CRITICAL();
}

/*
 void Rcd::setUnused(bool u) {
 taskENTER_CRITICAL();
 isrRcdUnused = u;
 taskEXIT_CRITICAL();
 }*/

bool Rcd::getEnabled() {
    bool b;
    taskENTER_CRITICAL();
    b = isrRcdEnabled;
    taskEXIT_CRITICAL();
    return b;
}

bool Rcd::getRcdFired() {
    // FIXME refactor this class and don't block all interrupts with
    // enter_critical all the time
    return isrRcdFired;
}
