/*
 * MainTask.h
 *
 *  Created on: 15.03.2021
 *      Author: cornelius
 */

#ifndef SRC_EVTASKS_MAINTASK_H_
#define SRC_EVTASKS_MAINTASK_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
	RESET_CAUSE_OPTION_BYTE_LOADER_RESET
} reset_cause_t;


void StartMainTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // SRC_EVTASKS_MAINTASK_H_
