/*
 * Adc.cpp
 *
 *  Created on: Mar 11, 2021
 *      Author: cornelius
 */

#include "Adc.h"

#include <cstring>

#include "cmsis_os.h"
#include "main.h"

#include "EVConfig.h"

Adc::Adc(ADC_HandleTypeDef *_adc) : adc(_adc) {

    evseCPHiIdx = 0;
    evseCPLoIdx = 0;

    supplyN12V = 0;
    temperature = 0;
    evsePP = 0;
    vrefint = 0;

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);

    HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED);
    registerCallback(IntType::HAL_ADC_ConvCpltCallback);

    // ADC runs continuously
    // 1 set of all channels
    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_result_dma, 6);
}

Adc::~Adc() { removeCallback(IntType::HAL_ADC_ConvCpltCallback); }

float Adc::getCarCPHi() {
#if 0
    uint16_t v;
    taskENTER_CRITICAL();
    v = carCPHi;
    taskEXIT_CRITICAL();
#ifdef EVDEVBOARD_R0_WORKAROUNDS
    return (((v / 4095.) - (20. / 45.)) * 10 + (20. / 45.)) * 3.3;
#else
    return (((v / 4095.) - (24.9. / 44.9)) * 10 + (24.9 / 44.9)) * 3.3;
#endif
#endif
    return 0;
}

float Adc::getCarCPLo() {
#if 0
    uint16_t v;
    taskENTER_CRITICAL();
    v = carCPLo;
    taskEXIT_CRITICAL();
#ifdef EVDEVBOARD_R0_WORKAROUNDS
    return (((v / 4095.) - (20. / 45.)) * 10 + (20. / 45.)) * 3.3;
#else
    return (((v / 4095.) - (24.9 / 44.9)) * 10 + (25. / 44.9)) * 3.3;
#endif
#endif
    return 0;
}

void Adc::getEvseCPHi(float *out) {
    int i = 0;
    uint16_t o[AVG];

    taskENTER_CRITICAL();
    memcpy(o, evseCPHi, AVG * sizeof(uint16_t));
    taskEXIT_CRITICAL();
    // printf("HI %u\n", o[1]);
    // convert to SI units in output buffer
    for (i = 0; i < AVG; i++) {
#ifdef EVDEVBOARD_R0_WORKAROUNDS
        out[i] =
            ((((float)o[i] / 4095.) - (20. / 45.)) * 10 + (20. / 45.)) * 3.3;
#else
        out[i] =
            ((((float)o[i] / 4095.) - (24.9 / 44.9)) * 10 + (24.9 / 44.9)) *
            3.3;
#endif
    }
}

void Adc::getEvseCPLo(float *out) {
    int i = 0;
    uint16_t o[AVG];

    taskENTER_CRITICAL();
    memcpy(o, evseCPLo, AVG * sizeof(uint16_t));
    taskEXIT_CRITICAL();
    // printf("LO %u\n", o[0]);
    // printf("PP %u\n", evsePP);
    // printf("EU %u\n", evseCPuntriggered);

    // convert to SI units in output buffer
    for (i = 0; i < AVG; i++) {
#ifdef EVDEVBOARD_R0_WORKAROUNDS
        out[i] =
            ((((float)o[i] / 4095.) - (20. / 45.)) * 10 + (20. / 45.)) * 3.3;
#else
        out[i] =
            ((((float)o[i] / 4095.) - (24.9 / 44.9)) * 10 + (24.9 / 44.9)) *
            3.3;
#endif
    }
}

float Adc::getEvsePP() {
    uint16_t v;
    taskENTER_CRITICAL();
    v = evsePP;
    taskEXIT_CRITICAL();
    return v / 4095. * 3.3;
}

float Adc::getSupply12V() {
    uint16_t v;
    taskENTER_CRITICAL();
    v = supply12V;
    taskEXIT_CRITICAL();
    return (v / 4095.) * 3.3 / 0.2;
}

float Adc::getSupplyN12V() {
    uint16_t v;
    taskENTER_CRITICAL();
    v = supplyN12V;
    taskEXIT_CRITICAL();
#ifdef EVDEVBOARD_R0_WORKAROUNDS
    return (((v / 4095.) - (20. / 45.)) * 10 + (20. / 45.)) * 3.3;
#else
    return (((v / 4095.) - (24.9 / 44.9)) * 10 + (24.9 / 44.9)) * 3.3;
#endif
}

float Adc::getTemperature() {
    uint16_t v;
    taskENTER_CRITICAL();
    v = temperature;
    taskEXIT_CRITICAL();
    return v;
}

/*
 void Adc::triggerCarCPHiISR() {
 // carCPHi = adc_result[0];
 }

 void Adc::triggerCarCPLoISR() {
 // carCPLo = adc_result[0];
 }
 */

void Adc::triggerEvseCPHiISR() {
    // HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
    // 1 set of all channels
    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_result_dma, 6);

    // should go to hi var. 0 would be no sampling at all.
    evseCPSampleTarget = 2;
}

void Adc::triggerEvseCPLoISR() {
    // 1 set of all channels
    // HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_result_dma, 6);

    evseCPSampleTarget = 1;
}

void Adc::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == adc) {
#if YETI_RELEASE == 0
        // HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);
        if (evseCPSampleTarget == 1) {
            evseCPLo[evseCPLoIdx] = adc_result_dma[0];
            if (++evseCPLoIdx >= AVG) {
                evseCPLoIdx = 0;
            }
        } else if (evseCPSampleTarget == 2) {
            evseCPHi[evseCPHiIdx] = adc_result_dma[0];
            if (++evseCPHiIdx >= AVG) {
                evseCPHiIdx = 0;
            }
        }
        evsePP = adc_result_dma[1];
        supply12V = adc_result_dma[2];
        supplyN12V = adc_result_dma[3];
        temperature = adc_result_dma[4];
        vrefint = adc_result_dma[5];
        evseCPSampleTarget = 0; // Not triggered at all
        // HAL_GPIO_WritePin(CABLE_32A_GPIO_Port, CABLE_32A_Pin,
        // GPIO_PIN_RESET);
    }
#elif YETI_RELEASE == 1
        // In R1 1 and 4 are switched in sequence. The first element in the
        // buffer is always wrong on this CPU for some reason. Temperature is on
        // [0] and currently does not work.
        // HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);
        if (evseCPSampleTarget == 1) {
            evseCPLo[evseCPLoIdx] = adc_result_dma[4];
            if (++evseCPLoIdx >= AVG) {
                evseCPLoIdx = 0;
            }
        } else if (evseCPSampleTarget == 2) {
            evseCPHi[evseCPHiIdx] = adc_result_dma[4];
            if (++evseCPHiIdx >= AVG) {
                evseCPHiIdx = 0;
            }
        }
        evseCPuntriggered = adc_result_dma[0];
        evsePP = adc_result_dma[1];
        supply12V = adc_result_dma[2];
        supplyN12V = adc_result_dma[3];
        temperature = adc_result_dma[0];
        vrefint = adc_result_dma[5];
        // printf("result %X %X %X %X %X %X\n", evseCPuntriggered, evsePP,
        //		supply12V, supplyN12V, temperature, vrefint);

        evseCPSampleTarget = 0; // Not triggered at all
        // HAL_GPIO_WritePin(CABLE_32A_GPIO_Port, CABLE_32A_Pin,
        // GPIO_PIN_RESET);
    }
#endif
}
