/*
 * SelfTest.cpp
 *
 *  Created on: Feb 27, 2021
 *      Author: cornelius
 */

#include "SelfTest.h"

#include <stdio.h>

#include "../protobuf/hi2lo.pb.h"
#include "../protobuf/lo2hi.pb.h"

#include "../EVDrivers/ADE7978.h"
#include "../EVDrivers/Adc.h"
#include "../EVDrivers/Charger.h"
#include "../EVDrivers/DOGM128W.h"
#include "../EVDrivers/Gpio.h"
#include "../EVDrivers/MgmtLink.h"
#include "../EVDrivers/PowerSwitch.h"
#include "../EVDrivers/Rcd.h"
#include "../EVDrivers/RemoteControl.h"
#include "../EVDrivers/SSD1322.h"
#include "../EVDrivers/SpiBus.h"
#include "EVConfig.h"

extern TIM_HandleTypeDef htim3, htim1, htim8;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
#if YETI_RELEASE == 1
extern SPI_HandleTypeDef hspi2;
#endif

#ifdef EMITEST_RELAIS_PWM
__STATIC_INLINE void Delay(__IO uint32_t micros) {
#if !defined(STM32F0xx)
    uint32_t start = DWT->CYCCNT;

    /* Go to number of cycles for system */
    micros *= (HAL_RCC_GetHCLKFreq() / 1000000);

    /* Delay till end */
    while ((DWT->CYCCNT - start) < micros)
        ;
#else
    /* Go to clock cycles */
    micros *= (SystemCoreClock / 1000000) / 5;

    /* Wait till done */
    while (micros--)
        ;
#endif
}
#endif

void StartSelfTestTask(void *argument) {
#ifdef EMITEST_RELAIS_PWM
    Gpio xl1(POWERSWITCH_L1_GPIO_Port, POWERSWITCH_L1_Pin);
    Gpio xl1mirror(MIRROR_L1_GPIO_Port, MIRROR_L1_Pin);
    Gpio xl2l3(POWERSWITCH_L2L3_GPIO_Port, POWERSWITCH_L2L3_Pin);
    Gpio xl2l3mirror(MIRROR_L2L3_GPIO_Port, MIRROR_L2L3_Pin);
    xl1.reset();
    xl2l3.reset();
    osDelay(2000);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    xl1.set();
    xl2l3.set();
    osDelay(100);
    while (true) { // reduce to 30% holding voltage
        xl1.set();
        xl2l3.set();
        Delay(1 * 3);
        xl1.reset();
        xl2l3.reset();
        Delay(1 * 7);
    }
#endif

    uint16_t totalrun = 0, totalfailed = 0;

    printf("---------------------------------\n");
    printf("Starting self test thread.\n");

#if YETI_RELEASE == 1
    static Gpio cfg(CFG_GPIO_Port, CFG_Pin);
    printf("CFG Jumper: %i\n", cfg.read());
#endif

    // SPI1 belongs to ABP2
    static SpiBus spi_bus(hspi1, HAL_RCC_GetPCLK2Freq());

#if YETI_RELEASE == 0
    static Gpio display_cs(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin);
    static Gpio display_dc(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin);
    static Gpio display_reset(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin);
    static SSD1322 _display(display_cs, display_dc, display_reset);
#elif YETI_RELEASE == 1
    static SpiBus spi_bus_display(hspi2, HAL_RCC_GetPCLK2Freq());
    static Gpio display_cs(DISPLAY_CS_LED1_GPIO_Port, DISPLAY_CS_LED1_Pin);
    static Gpio display_dc(DISPLAY_DC_LED2_GPIO_Port, DISPLAY_DC_LED2_Pin);
    static Gpio display_reset(DISPLAY_RESET_LED3_GPIO_Port,
                              DISPLAY_RESET_LED3_Pin);
    static DOGM128W _display(display_cs, display_dc, display_reset);
    spi_bus_display.initialize({&_display});
    _display.run();
#endif

    Display *display = &_display;


    Gpio l1mirror(MIRROR_L1_GPIO_Port, MIRROR_L1_Pin);
    Gpio l2l3mirror(MIRROR_L2L3_GPIO_Port, MIRROR_L2L3_Pin);
    PowerSwitch ps(&htim8, l1mirror, l2l3mirror);

    static Gpio meter_cs(METER_CS_GPIO_Port, METER_CS_Pin);
    static Gpio int1(PM_IRQ1_GPIO_Port, PM_IRQ1_Pin);
    static ADE7978 power_meter(meter_cs, int1, &ps);

#if YETI_RELEASE == 0
    spi_bus.initialize({&_display, &power_meter});
#elif YETI_RELEASE == 1
    // FIXME: add second display SPI and FRAM here
    spi_bus.initialize({&power_meter});
#endif

    power_meter.softReset();
    power_meter.setup();

    // Sending some static strings to the display
    display->printText("pionix @ everest", 0, 0, Display::CENTER);
    display->printText("Date: 12.01.21", 0, 1);
    display->printText("E: 50.3KWh", 0, 1, Display::RIGHT);
    display->printText("Time: 12:23:11", 0, 2);
    display->printText("C: 34A", 0, 2, Display::RIGHT);
    display->printText("!@#$%^&*()_+={}", 0, 3, Display::CENTER);


    // Self test Power Relais:
    // switch single and three phase on and off and verify that mirror contacts
    // react as needed.

    while(1) {
    	ps.switchOnThreePhase();
    	osDelay(5000);
    	ps.switchOff();
    	osDelay(2000);

    }

    if (!ps.executeSelfTest())
        totalfailed++;
    totalrun++;

    printf("\n\n\n");

    // RCD self test
    //
    // A residual current is induced into the sensor via a testing winding.
    // Tests if residual current is being measured and if RCD triggers a fault.

    Gpio testout(RCD_TEST_GPIO_Port, RCD_TEST_Pin);
#if YETI_RELEASE == 0
    Gpio errorin(RCD_ERROR_GPIO_Port, RCD_ERROR_Pin);
#elif YETI_RELEASE == 1
    Gpio errorin(RCD_DC_ERROR_GPIO_Port, RCD_DC_ERROR_Pin);
#endif

    Gpio pwmin(RCD_PWM_IN_GPIO_Port, RCD_PWM_IN_Pin);

    Rcd rcd(testout, errorin, pwmin, &htim3, &ps);
    if (!rcd.executeSelfTest())
        totalfailed++;
    totalrun++;
    osDelay(1000);
    printf("\n\n\n");

    Adc adc(&hadc1);

    // read ADC values for a while
    for (int i = 0; i < 10; i++) {
        adc.triggerEvseCPHiISR();
        osDelay(100);
        printf("PP: %2.2f 12V: %2.2f, -12V: %2.2f\n", adc.getEvsePP(),
               adc.getSupply12V(), adc.getSupplyN12V());
    }

    // Charger test: connect CP of sim port externally to CP of EVSE with cable
    // to make this test work!

#if YETI_RELEASE == 0
    static Charger charger(&htim1, ps, adc, rcd, display, power_meter, NULL);
#elif YETI_RELEASE == 1
    static Gpio cp_enable(CP_ENABLE_GPIO_Port, CP_ENABLE_Pin);
    static ControlPilot_HAL control_pilot_hal(&htim1, adc, &cp_enable);
    static ControlPilot control_pilot(control_pilot_hal, ps, rcd);
    static Charger charger(control_pilot, display, power_meter);
#endif

    // We need to set a current at least once
    charger.setMaxCurrent(6);
    charger.setHasVentilation(false);

    charger.setThreePhases(true);
    charger.run(); // spins off thread and returns immediately
    osDelay(200);
    charger.enable(); // activates running charger to accept cars

#if YETI_RELEASE == 0
    Gpio carAB(CAR_AB_GPIO_Port, CAR_AB_Pin);
    Gpio carC(CAR_C_GPIO_Port, CAR_C_Pin);
    Gpio carD(CAR_D_GPIO_Port, CAR_D_Pin);
    Gpio carE(CAR_E_GPIO_Port, CAR_E_Pin);
    Gpio carDF(CAR_DF_GPIO_Port, CAR_DF_Pin);
    Gpio cable13A(CABLE_13A_GPIO_Port, CABLE_13A_Pin);
    Gpio cable20A(CABLE_20A_GPIO_Port, CABLE_20A_Pin);
    Gpio cable32A(CABLE_32A_GPIO_Port, CABLE_32A_Pin);
    CarSim carsim(carAB, carC, carD, carE, carDF, cable13A, cable20A, cable32A,
                  charger);

    // this test with while(1) below was used to test PWM rising/falling times
    // carsim.runTestSequence("A06V12.TA1.V09.TB2.V06.TC2");
#elif YETI_RELEASE == 1
    /* osDelay(500);
     charger.setTestmode(1);
     osDelay(500);
     charger.setTestmode(2);
     charger.setTestPWM(4. / 100 * 48000);
     // charger.setTestPWM(95./100*48000);
     while (1) {
     }*/
#endif

    printf("Starting Communication Link...\n");
    // enable communication to hi level stack
    static MgmtLink link(&huart2);
    static RemoteControl rc(charger, link);
    rc.runInCurrent();

    /*while (1) {
     osDelay(1000);
     };*/

#if YETI_RELEASE == 0
    // test sequence A.4.7.2 typical charging sequence
    if (!carsim.runTestSequence(
            "A06V12.TA1.V09.TB2.V06.TC2.V09.TB2.V06.TC2.A16..V09.TB2.V12.TA1"))
        totalfailed++;
    totalrun++;

    // ventilation charging sequence
    if (!carsim.runTestSequence("A06V12.TA1.V09.TB2.V03.TEV.V12.TA1"))
        totalfailed++;
    totalrun++;

    // test sequence A.4.7.3 simplified charging sequence
    if (!carsim.runTestSequence("A16V12.TA1.V06.TC2.V12.TA1"))
        totalfailed++;
    totalrun++;

    // test sequence with diode fault
    if (!carsim.runTestSequence("A06V12.TA1.V09.TB2.V06.TC2.EDF.TDF.V12.TA1"))
        totalfailed++;
    totalrun++;

    // test sequence with CP-PE short Error E
    if (!carsim.runTestSequence("A06V12.TA1.V09.TB2.V06.TC2.EEE.TEE.V12.TA1"))
        totalfailed++;
    totalrun++;

#endif

    osDelay(1000);
    printf("\n\n\n--------------------------------------------------------\n");
    printf("Summary: %u tests run, %u failed.\n", totalrun, totalfailed);
    if (totalfailed == 0) {
        printf("SUCCESS\n");
    } else {
        printf("FAILURE");
    }
    printf("\n--------------------------------------------------------\n");

    while (1) {
        // printf ("osThreadGetCount %u\n",osThreadGetCount());
        osDelay(1000);
    }
}
