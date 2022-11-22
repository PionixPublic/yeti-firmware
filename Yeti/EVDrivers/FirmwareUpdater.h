/*
 * FirmwareUpdater.h
 *
 *  Created on: 20.08.2021
 *      Author: cornelius
 */

#ifndef EVDRIVERS_FIRMWAREUPDATER_H_
#define EVDRIVERS_FIRMWAREUPDATER_H_

#include "EVConfig.h"
#include "cmsis_os.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile int reset_flags;

void restartInBootLoaderMode();
void restartInBootLoaderMode_Step2();

#ifdef __cplusplus
}
#endif

#endif /* EVDRIVERS_FIRMWAREUPDATER_H_ */
