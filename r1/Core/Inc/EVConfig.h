/*
 * EVConfig.h
 *
 *  Created on: Mar 11, 2021
 *      Author: cornelius
 */

#ifndef SRC_EVDRIVERS_EVCONFIG_H_
#define SRC_EVDRIVERS_EVCONFIG_H_

#define YETI_RELEASE 1

// define to compile for self test mode instead of normal working mode.
// Do NOT connect car or AC voltage in self test mode!
//#define SELFTEST_MODE
//#define EMITEST_RELAIS_PWM

// define this to enable workarounds for HW bugs in R0 devboard
//#define EVDEVBOARD_R0_WORKAROUNDS

// One or Three phase on startup?
#define USE_THREE_PHASES true

// NOTE: In the following countries automatic reclosing of protection means is
// not allowed: DK, UK, FR, CH uncomment for these countries
//#define DISABLE_RCD_RECLOSING

// NOTE: This should be disabled in production environment and replaced with a secure boot loader
#define ALLOW_ROM_BOOTLOADER

// Ignore TESLA's funny CP sequence B->C->DF->B 5 times
//#define IGNORE_TESLA_SPECIAL_SEQUENCE

/*
 * Enable debug printf's in the threads:
 * Be careful with enableing printf's in multiple threads. This can hard fault
 * probably due to still buggy malloc implementation that is called in printf.
 * Disable all for release!
 * */

//#define REMOTECONTROL_CPP_ENABLE_PRINTF
//#define ADE7978_CPP_ENABLE_PRINTF
//#define CHARGER_CPP_ENABLE_PRINTF
//#define MGMTLINK_CPP_ENABLE_PRINTF
//#define RCD_CPP_ENABLE_PRINTF
//#define SPIBUS_CPP_ENABLE_PRINTF
//#define FINE_GRAIN_DEBUG_PRINTF

#endif // SRC_EVDRIVERS_EVCONFIG_H_


