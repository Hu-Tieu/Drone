#ifndef CONTROLLER_SCHEDULER_TASKS_H
#define CONTROLLER_SCHEDULER_TASKS_H

#include <stdint.h>
#include "system_mode.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Forward declare ControlSystem_t to avoid circular includes. */
struct ControlSystem_t;


/* SystemMode_t and ModeConfig_t are declared in system_mode.h */
// Execution context passed to every scheduler task handler.
typedef struct {
    void *sys; /* opaque pointer to ControlSystem_t; cast in .c */
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
void Scheduler_ForceSafetyMode(uint8_t allow_rearm);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_SCHEDULER_TASKS_H
