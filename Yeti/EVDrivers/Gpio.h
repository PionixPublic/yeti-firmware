/*
 * Gpio.h
 *
 *  Created on: 26.02.2021
 *      Author: cornelius
 *
 * Basic C++ abstraction for GPIOs based on HAL driver.
 * Does not cover init yet, this is done in main.c via auto generated code
 * from .ioc file.
 */

#ifndef SRC_EVDRIVERS_GPIO_H_
#define SRC_EVDRIVERS_GPIO_H_

#include "stm32g4xx_hal.h"

class Gpio {
public:
    Gpio(GPIO_TypeDef *_port, uint16_t _pin);
    virtual ~Gpio();
    void set();
    void reset();
    bool read();

    uint16_t getPin();
    GPIO_TypeDef *getPort();

private:
    GPIO_TypeDef *port;
    uint16_t pin;
};

#endif // SRC_EVDRIVERS_GPIO_H_
