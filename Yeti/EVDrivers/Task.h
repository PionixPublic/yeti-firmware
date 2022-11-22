/*
 * Task.h
 *
 *  Created on: Mar 12, 2021
 *      Author: cornelius
 */

#ifndef SRC_EVDRIVERS_TASK_H_
#define SRC_EVDRIVERS_TASK_H_

#include <stdio.h>

#include "cmsis_os.h"

class Task {
public:
    Task(const char *name, uint32_t _stackSize,
         osPriority_t prio = osPriorityNormal);
    virtual ~Task();

    // start internal loop (main function) in new task and return
    void run();

    // start internal loop (main function) in current task
    // (should not return)
    void runInCurrent();

    // suspend/resume only make sense if a new thread was started with run().
    // Ignored if runInCurrent() was used
    void suspend();
    void resume();

private:
    osThreadId_t taskHandle;
    static void startTask(void *argument);
    virtual void main() = 0;
    const char *taskName;
    uint32_t stackSize;
    osPriority_t priority;
    volatile bool suspended;
};

#endif // SRC_EVDRIVERS_TASK_H_
