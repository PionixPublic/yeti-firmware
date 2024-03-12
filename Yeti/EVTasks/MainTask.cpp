/*
 * MainTask.cpp
 *
 *  Created on: 15.03.2021
 *      Author: cornelius
 */

#include "MainTask.h"

#include <stdio.h>

#include "../EVDrivers/ADE7978.h"
#include "../EVDrivers/Adc.h"
#include "../EVDrivers/ControlPilot.h"
#include "../EVDrivers/FirmwareUpdater.h"
#include "../EVDrivers/Gpio.h"
#include "../EVDrivers/PowerSwitch.h"
#include "../EVDrivers/Rcd.h"
#include "../EVDrivers/RemoteControlRX.h"
#include "../EVDrivers/RemoteControlTX.h"
#include "../EVDrivers/SpiBus.h"
#include "EVConfig.h"

extern TIM_HandleTypeDef htim3, htim1, htim8;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
#if YETI_RELEASE == 1
extern SPI_HandleTypeDef hspi2;
#endif
volatile extern int reset_flags;
float hard_limit;
int reset_cause_int;

/// @brief  Possible STM32 system reset causes
reset_cause_t reset_cause_get(void);
const char* reset_cause_get_name(reset_cause_t reset_cause);

void StartMainTask(void *argument) {

	reset_cause_t reset_cause = reset_cause_get();
	reset_cause_int = (int) reset_cause;
	printf("\n------------------------------------\nReset cause: %s\n",
			reset_cause_get_name(reset_cause));

	// SPI1 belongs to ABP2
	static SpiBus spi_bus(hspi1, HAL_RCC_GetPCLK2Freq());

	// read out config jumper and set a hard current limit
	static Gpio current_limit_cfg(CFG_GPIO_Port, CFG_Pin);

	if (current_limit_cfg.read())
		hard_limit = 16.;
	else
		hard_limit = 32.;

	printf("Starting PowerSwitch..\n");
	// Power Switch driver
	static Gpio l1mirror(MIRROR_L1_GPIO_Port, MIRROR_L1_Pin);
	static Gpio l2l3mirror(MIRROR_L2L3_GPIO_Port, MIRROR_L2L3_Pin);
	static PowerSwitch ps(&htim8, l1mirror, l2l3mirror);

	printf("Starting Powermeter...\n");
	static Gpio meter_cs(METER_CS_GPIO_Port, METER_CS_Pin);
	static Gpio int1(PM_IRQ1_GPIO_Port, PM_IRQ1_Pin);
	static ADE7978 power_meter(meter_cs, int1, &ps);

	// FIXME: add FRAM here
	spi_bus.initialize( { &power_meter });

	power_meter.softReset();
	power_meter.setup();
	// switch off immediately if current is 30% over limit
	power_meter.set_over_current_limit(hard_limit * 1.5);

	printf("Starting RCD...\n");
	// RCD driver
	static Gpio testout(RCD_TEST_GPIO_Port, RCD_TEST_Pin);
	static Gpio errorin(RCD_DC_ERROR_GPIO_Port, RCD_DC_ERROR_Pin);

	static Gpio pwmin(RCD_PWM_IN_GPIO_Port, RCD_PWM_IN_Pin);
	static Rcd rcd(testout, errorin, pwmin, &htim3, &ps);
#ifdef RCD_UNUSED
    rcd.setUnused(true);
#endif
	printf("Starting ADC...\n");
	// ADC driver
	static Adc adc(&hadc1);

	// osDelay(7000);
	// If we start RCD self test too early after power on, it will fail
	// Execute RCD self test at least once on startup
#if 0
    if (!rcd.executeSelfTest()) {
        printf("DEAD BEEF: RCD self test failed.\n");
        while (1) {
            osDelay(50);
        };
    } else {
        printf("RCD self test successful.\n");
    }
#endif

	printf("Starting ControlPilot...\n");

	static Gpio cp_enable(CP_ENABLE_GPIO_Port, CP_ENABLE_Pin);
	static ControlPilot_HAL control_pilot_hal(&htim1, adc, &cp_enable);

	printf("Starting MgmtLink...\n");
	// enable communication to hi level stack
	static MgmtLink link(&huart2);

		printf("Start RemoteControl...\n");
	static RemoteControlTX rc_tx(link);

	static ControlPilot control_pilot(control_pilot_hal, rc_tx, ps, rcd);

	control_pilot.run(); // spins off thread and returns immediately

	static RemoteControlRX rc_rx(control_pilot, rc_tx, link);

	printf("Start RemoteControl thread...\n");
	rc_rx.runInCurrent();
	// we should not reach this point
}

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause

#define RCC_FLAG_OBLRST                ((CSR_REG_INDEX << 5U) | RCC_CSR_OBLRSTF_Pos)   /*!< Option Byte Loader reset flag */
#define RCC_FLAG_PINRST                ((CSR_REG_INDEX << 5U) | RCC_CSR_PINRSTF_Pos)   /*!< PIN reset flag */
#define RCC_FLAG_BORRST                ((CSR_REG_INDEX << 5U) | RCC_CSR_BORRSTF_Pos)   /*!< BOR reset flag */
#define RCC_FLAG_SFTRST                ((CSR_REG_INDEX << 5U) | RCC_CSR_SFTRSTF_Pos)   /*!< Software Reset flag */
#define RCC_FLAG_IWDGRST               ((CSR_REG_INDEX << 5U) | RCC_CSR_IWDGRSTF_Pos)  /*!< Independent Watchdog reset flag */
#define RCC_FLAG_WWDGRST               ((CSR_REG_INDEX << 5U) | RCC_CSR_WWDGRSTF_Pos)  /*!< Window watchdog reset flag */
#define RCC_FLAG_LPWRRST               ((CSR_REG_INDEX << 5U) | RCC_CSR_LPWRRSTF_Pos)  /*!< Low-Power reset flag */

reset_cause_t reset_cause_get(void) {
	reset_cause_t reset_cause;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		reset_cause = RESET_CAUSE_LOW_POWER_RESET;
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
		reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		// This reset is induced by calling the ARM CMSIS
		// `NVIC_SystemReset()` function!
		reset_cause = RESET_CAUSE_SOFTWARE_RESET;
	}
	/*else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
	 {
	 reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
	 }*/
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)) {
		reset_cause = RESET_CAUSE_OPTION_BYTE_LOADER_RESET;
	} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
	}
	// Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
	// ensure first that the reset cause is NOT a POR/PDR reset. See note
	// below.
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
		reset_cause = RESET_CAUSE_BROWNOUT_RESET;
	} else {
		reset_cause = RESET_CAUSE_UNKNOWN;
	}

	// Clear all the reset flags or else they will remain set during future
	// resets until system power is fully removed.
	__HAL_RCC_CLEAR_RESET_FLAGS();

	return reset_cause;
}

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
// Controller (RCC) header files, such as
// "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
// etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
// the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
// (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
// See the doxygen just above their definition for the
// `__HAL_RCC_GET_FLAG()` macro to see this:
//      "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
//      Reset flag will *also* be set in the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
// first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
// reset cause is NOT a POR/PDR reset.

/// @brief      Obtain the system reset cause as an ASCII-printable name string
///             from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system
///             reset cause
const char* reset_cause_get_name(reset_cause_t reset_cause) {
	const char *reset_cause_name = "TBD";

	switch (reset_cause) {
	case RESET_CAUSE_UNKNOWN:
		reset_cause_name = "UNKNOWN";
		break;
	case RESET_CAUSE_LOW_POWER_RESET:
		reset_cause_name = "LOW_POWER_RESET";
		break;
	case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
		reset_cause_name = "WINDOW_WATCHDOG_RESET";
		break;
	case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
		reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
		break;
	case RESET_CAUSE_SOFTWARE_RESET:
		reset_cause_name = "SOFTWARE_RESET";
		break;
	case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
		reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
		break;
	case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
		reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
		break;
	case RESET_CAUSE_BROWNOUT_RESET:
		reset_cause_name = "BROWNOUT_RESET (BOR)";
		break;
	case RESET_CAUSE_OPTION_BYTE_LOADER_RESET:
		reset_cause_name = "OPTION BYTE LOADER (OBLRST)";
		break;
	}

	return reset_cause_name;
}

