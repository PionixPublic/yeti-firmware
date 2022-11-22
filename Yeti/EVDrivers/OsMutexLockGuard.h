/*
 * OsMutexLockGuard.h
 *
 *  Created on: 29.10.2021
 *      Author: cornelius
 */

#ifndef EVDRIVERS_OSMUTEXLOCKGUARD_H_
#define EVDRIVERS_OSMUTEXLOCKGUARD_H_

#include "cmsis_os.h"

class OsMutexLockGuard {
public:
    OsMutexLockGuard(osMutexId_t &mutex);
    ~OsMutexLockGuard();

private:
    osMutexId_t &mutex;
};

#endif /* EVDRIVERS_OSMUTEXLOCKGUARD_H_ */
