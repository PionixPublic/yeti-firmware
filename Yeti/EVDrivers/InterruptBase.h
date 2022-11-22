/*
 * InterruptBase.h
 *
 *  Created on: 02.03.2021
 *      Author: cornelius
 *
 * Base class for all objects that can receive interrupt handler callbacks.
 * Objects must register via registerCallback() to actually receive a
 * callback to virtual overloaded member function.
 *
 * The lists of registrations is global and must only be edited through
 * registerCallback() and removeCallback().
 *
 * The actual Interrupt routines are in stm32g4xx_hal.h. HAL already
 * diffuses individual callbacks for different types (e.g. CC / overflow events
 * of Timer interrupt) to global callback functions (implemented at the end of
 * the source file). These global callback functions iterate through the
 * registration list and call the virtual callback handler of the actual C++
 * object.
 */

#ifndef SRC_EVDRIVERS_INTERRUPTBASE_H_
#define SRC_EVDRIVERS_INTERRUPTBASE_H_

#include <cstdio>
#include <list>

#include "stm32g4xx_hal.h"

class InterruptBase {
public:
    // pure virtual Callbacks
    virtual void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){};
    virtual void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){};
    virtual void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){};
    virtual void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){};
    virtual void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){};
    virtual void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){};
    virtual void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){};
    virtual void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){};
    virtual void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){};
    virtual void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi){};
    virtual void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){};
    virtual void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){};

protected:
    enum class IntType {
        HAL_TIM_IC_CaptureCallback,
        HAL_TIM_PeriodElapsedCallback,
        HAL_GPIO_EXTI_Callback,
        HAL_ADC_ConvCpltCallback,
        HAL_TIM_PWM_PulseFinishedCallback,
        HAL_TIM_OC_DelayElapsedCallback,
        HAL_UART_RxHalfCpltCallback,
        HAL_UART_RxCpltCallback,
        HAL_UART_TxCpltCallback,
        HAL_SPI_TxHalfCpltCallback,
        HAL_SPI_TxCpltCallback,
        HAL_UART_ErrorCallback
    };
    bool registerCallback(IntType type);
    bool removeCallback(IntType type);
};

#endif // SRC_EVDRIVERS_INTERRUPTBASE_H_
