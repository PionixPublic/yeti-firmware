/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 20.08.2021
 *      Author: cornelius
 */

#include "FirmwareUpdater.h"
typedef void (*pFunction)(void);
pFunction JumpToApplication;

// only works if the no_init section is correctly defined in .ld file
volatile int reset_flags __attribute__ ((section (".no_init")));

void restartInBootLoaderMode() {
#ifdef ALLOW_ROM_BOOTLOADER
    reset_flags = 0x12ABCDEF;
    NVIC_SystemReset();
#endif
}

void restartInBootLoaderMode_Step2() {
#ifdef ALLOW_ROM_BOOTLOADER
#if YETI_RELEASE == 1
	reset_flags = 0;
    void (*SysMemBootJump)(void);
    volatile uint32_t addr = 0x1FFF0000;
    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __disable_irq();
    __DSB();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    __DSB();
    __ISB();
    SysMemBootJump = (void (*)(void))(*((uint32_t *)(addr + 4)));
    __set_MSP(*(uint32_t *)addr);
    SysMemBootJump();
#endif
#endif
}
