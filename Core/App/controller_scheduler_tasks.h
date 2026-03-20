#ifndef CONTROLLER_SCHEDULER_TASKS_H
#define CONTROLLER_SCHEDULER_TASKS_H

#include <stdint.h>
#include "controller_main.h"
#include "Receiver/receiver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Execution context passed to every scheduler task handler.
typedef struct {
    ControlSystem_t *sys;
} SchedulerContext_t;

// Handler function type (single context pointer).
typedef void (*SchedulerTaskHandler_t)(SchedulerContext_t *ctx);

typedef struct {
    uint32_t flag;
    SchedulerTaskHandler_t handler;
} SchedulerTask_t;

// Expose the task table and count
extern const SchedulerTask_t scheduler_tasks[];
extern const int num_scheduler_tasks;

// Main entry to run all tasks for set flags
void Scheduler_RunTasks(uint32_t *pending_flags, SchedulerContext_t *ctx);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_SCHEDULER_TASKS_H
