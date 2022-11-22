/*
 * InterruptBase.cpp
 *
 *  Created on: 02.03.2021
 *      Author: cornelius
 */

#include "InterruptBase.h"

#include "main.h"

// GLOBAL list of registered objects that would like to receive callbacks
static std::list<InterruptBase *> list_HAL_TIM_IC_CaptureCallback;
static std::list<InterruptBase *> list_HAL_TIM_PeriodElapsedCallback;
static std::list<InterruptBase *> list_HAL_GPIO_EXTI_Callback;
static std::list<InterruptBase *> list_HAL_ADC_ConvCpltCallback;
static std::list<InterruptBase *> list_HAL_TIM_PWM_PulseFinishedCallback;
static std::list<InterruptBase *> list_HAL_TIM_OC_DelayElapsedCallback;
static std::list<InterruptBase *> list_HAL_UART_RxHalfCpltCallback;
static std::list<InterruptBase *> list_HAL_UART_RxCpltCallback;
static std::list<InterruptBase *> list_HAL_UART_TxCpltCallback;
static std::list<InterruptBase *> list_HAL_SPI_TxHalfCpltCallback;
static std::list<InterruptBase *> list_HAL_SPI_TxCpltCallback;
static std::list<InterruptBase *> list_HAL_UART_ErrorCallback;

bool InterruptBase::registerCallback(IntType type) {
    bool success = false;
    switch (type) {
    case IntType::HAL_TIM_IC_CaptureCallback:
        list_HAL_TIM_IC_CaptureCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_TIM_PeriodElapsedCallback:
        list_HAL_TIM_PeriodElapsedCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_GPIO_EXTI_Callback:
        list_HAL_GPIO_EXTI_Callback.push_back(this);
        success = true;
        break;
    case IntType::HAL_ADC_ConvCpltCallback:
        list_HAL_ADC_ConvCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_TIM_PWM_PulseFinishedCallback:
        list_HAL_TIM_PWM_PulseFinishedCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_TIM_OC_DelayElapsedCallback:
        list_HAL_TIM_OC_DelayElapsedCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_UART_RxHalfCpltCallback:
        list_HAL_UART_RxHalfCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_UART_RxCpltCallback:
        list_HAL_UART_RxCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_UART_TxCpltCallback:
        list_HAL_UART_TxCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_SPI_TxHalfCpltCallback:
        list_HAL_SPI_TxHalfCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_SPI_TxCpltCallback:
        list_HAL_SPI_TxCpltCallback.push_back(this);
        success = true;
        break;
    case IntType::HAL_UART_ErrorCallback:
        list_HAL_UART_ErrorCallback.push_back(this);
        success = true;
        break;
    }
    return success;
}

bool InterruptBase::removeCallback(IntType type) {
    bool success = false;
    switch (type) {
    case IntType::HAL_TIM_IC_CaptureCallback:
        list_HAL_TIM_IC_CaptureCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_TIM_PeriodElapsedCallback:
        list_HAL_TIM_PeriodElapsedCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_GPIO_EXTI_Callback:
        list_HAL_GPIO_EXTI_Callback.remove(this);
        success = true;
        break;
    case IntType::HAL_ADC_ConvCpltCallback:
        list_HAL_ADC_ConvCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_TIM_PWM_PulseFinishedCallback:
        list_HAL_TIM_PWM_PulseFinishedCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_TIM_OC_DelayElapsedCallback:
        list_HAL_TIM_OC_DelayElapsedCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_UART_RxHalfCpltCallback:
        list_HAL_UART_RxHalfCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_UART_RxCpltCallback:
        list_HAL_UART_RxCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_UART_TxCpltCallback:
        list_HAL_UART_TxCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_SPI_TxHalfCpltCallback:
        list_HAL_SPI_TxHalfCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_SPI_TxCpltCallback:
        list_HAL_SPI_TxCpltCallback.remove(this);
        success = true;
        break;
    case IntType::HAL_UART_ErrorCallback:
        list_HAL_UART_ErrorCallback.remove(this);
        success = true;
        break;
    }
    return success;
}

// GLOBAL functions for actual callbacks from HAL Interrupt handler from
// stm32g4xx_hal.h

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // run all callbacks on registered objects
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_TIM_IC_CaptureCallback.begin();
         it != list_HAL_TIM_IC_CaptureCallback.end(); ++it) {
        (*it)->HAL_TIM_IC_CaptureCallback(htim);
    }
}

// Note special __ naming here: this is already defined in main.c as TIM2
// overflow is used for STM32 non RTOS framework things. A call to this function
// needs to be added to main.c implementation in user section.
extern "C" void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // run all callbacks on registered objects
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_TIM_PeriodElapsedCallback.begin();
         it != list_HAL_TIM_PeriodElapsedCallback.end(); ++it) {
        (*it)->HAL_TIM_PeriodElapsedCallback(htim);
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_GPIO_EXTI_Callback.begin();
         it != list_HAL_GPIO_EXTI_Callback.end(); ++it) {
        (*it)->HAL_GPIO_EXTI_Callback(GPIO_Pin);
    }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // Conversion Complete & DMA Transfer Complete As Well
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_ADC_ConvCpltCallback.begin();
         it != list_HAL_ADC_ConvCpltCallback.end(); ++it) {
        (*it)->HAL_ADC_ConvCpltCallback(hadc);
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_TIM_PWM_PulseFinishedCallback.begin();
         it != list_HAL_TIM_PWM_PulseFinishedCallback.end(); ++it) {
        (*it)->HAL_TIM_PWM_PulseFinishedCallback(htim);
    }
}

extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_TIM_OC_DelayElapsedCallback.begin();
         it != list_HAL_TIM_OC_DelayElapsedCallback.end(); ++it) {
        (*it)->HAL_TIM_OC_DelayElapsedCallback(htim);
    }
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_UART_RxHalfCpltCallback.begin();
         it != list_HAL_UART_RxHalfCpltCallback.end(); ++it) {
        (*it)->HAL_UART_RxHalfCpltCallback(huart);
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_UART_RxCpltCallback.begin();
         it != list_HAL_UART_RxCpltCallback.end(); ++it) {
        (*it)->HAL_UART_RxCpltCallback(huart);
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_UART_TxCpltCallback.begin();
         it != list_HAL_UART_TxCpltCallback.end(); ++it) {
        (*it)->HAL_UART_TxCpltCallback(huart);
    }
}

extern "C" void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_SPI_TxHalfCpltCallback.begin();
         it != list_HAL_SPI_TxHalfCpltCallback.end(); ++it) {
        (*it)->HAL_SPI_TxHalfCpltCallback(hspi);
    }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_SPI_TxCpltCallback.begin();
         it != list_HAL_SPI_TxCpltCallback.end(); ++it) {
        (*it)->HAL_SPI_TxCpltCallback(hspi);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    std::list<InterruptBase *>::iterator it;
    for (it = list_HAL_UART_ErrorCallback.begin();
         it != list_HAL_UART_ErrorCallback.end(); ++it) {
        (*it)->HAL_UART_ErrorCallback(huart);
    }
}
