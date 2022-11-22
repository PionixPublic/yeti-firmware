/*
 * Adc.h
 *
 *  Created on: Mar 11, 2021
 *      Author: cornelius
 *
 * Simple ADC abstraction.
 *
 * ADC is configured to sample all used channels.
 * Results are transferred using DMA to buffer.
 * When DMA finished, triggered data is copied to vars.
 * TODO: recalibration at regular intervals if temperature changes?
 * TODO: internal temperature reading see RM0440 pg 684
 *
 */

#ifndef SRC_EVDRIVERS_ADC_H_
#define SRC_EVDRIVERS_ADC_H_

#include "InterruptBase.h"

class Adc : private InterruptBase {
public:
    Adc(ADC_HandleTypeDef *_adc);
    virtual ~Adc();

    float getCarCPHi();
    float getCarCPLo();

    void getEvseCPHi(float *out);
    void getEvseCPLo(float *out);
    float getEvsePP();

    float getSupply12V();
    float getSupplyN12V();
    float getTemperature();

    // Call trigger from ISR only
    // calling ISR prio must be <= ADC DMA IRQ prio
    /*  void triggerCarCPHiISR();
      void triggerCarCPLoISR();
      */
    void triggerEvseCPHiISR();
    void triggerEvseCPLoISR();

    virtual void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

    static constexpr uint8_t AVG = 10;

private:
    ADC_HandleTypeDef *adc;
    // DMA buffer for results
#pragma pack(4)
    uint16_t adc_result_dma[7];
    // buffer for triggered values
    // uint16_t carCPHi, carCPLo;
    uint16_t evseCPHi[AVG], evseCPLo[AVG];
    volatile uint8_t evseCPHiIdx, evseCPLoIdx;
    volatile uint16_t supply12V, supplyN12V, temperature, evsePP, vrefint;
    volatile uint8_t evseCPSampleTarget;
    volatile uint16_t evseCPuntriggered;
};

#endif // SRC_EVDRIVERS_ADC_H_
