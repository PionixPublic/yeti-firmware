/*
 * OsMutexLockGuard.cpp
 *
 *  Created on: 29.10.2021
 *      Author: cornelius
 */

#include <EVDrivers/OsMutexLockGuard.h>

OsMutexLockGuard::OsMutexLockGuard(osMutexId_t &mutex) : mutex(mutex) {
    osMutexWait(mutex, osWaitForever);
}

OsMutexLockGuard::~OsMutexLockGuard() { osMutexRelease(mutex); }
