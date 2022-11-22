/*
 * Thread.cpp
 *
 *  Created on: Mar 12, 2021
 *      Author: cornelius
 */

#include "Task.h"

Task::Task(const char *name, uint32_t _stackSize, osPriority_t prio) :
    taskName(name), stackSize(_stackSize) {
    priority = prio;
    taskHandle = NULL;
    suspended = false;
}

Task::~Task() { osThreadTerminate(taskHandle); }

void Task::run() {
    const osThreadAttr_t task_attributes = {
        .name = taskName,
        .attr_bits = 0,
        .cb_mem = NULL,
        .cb_size = 0,
        .stack_mem = NULL,
        .stack_size = stackSize,
        .priority = priority,
        .tz_module = 0,
        .reserved = 0,
    };
    taskHandle = osThreadNew(Task::startTask, (void *)this, &task_attributes);
}

void Task::runInCurrent() { main(); }

void Task::startTask(void *argument) { ((Task *)argument)->main(); }

void Task::suspend() {
    if (taskHandle) {
        suspended = true;
        osThreadSuspend(taskHandle);
    }
}

void Task::resume() {
    if (taskHandle && suspended) {
        osThreadResume(taskHandle);
        suspended = false;
    }
}
