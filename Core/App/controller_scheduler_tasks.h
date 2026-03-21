#ifndef CONTROLLER_SCHEDULER_TASKS_H
#define CONTROLLER_SCHEDULER_TASKS_H

#include <stdint.h>
#include "controller_main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SystemMode_t
{
    SYSTEM_MODE_SAFETY = 0,        // CH4=0, CH5=0
    SYSTEM_MODE_SAFETY1,           // CH4=0, CH5=1
    SYSTEM_MODE_RESERVED,          // CH4=0, CH5=2
    SYSTEM_MODE_TEST,              // CH4=1, CH5=0
    SYSTEM_MODE_IDLE,              // CH4=1, CH5=1
    SYSTEM_MODE_FLIGHT,            // CH4=1, CH5=2
} SystemMode_t;

typedef struct {
    uint8_t enable_rate;
    uint8_t enable_angle;
    uint8_t enable_velocity;
    uint8_t enable_position;
    uint8_t enable_motor_output;
} ModeConfig_t;

static ModeConfig_t mode_table[] = {
    [SYSTEM_MODE_SAFETY] = {
        .enable_rate = 0,
        .enable_angle = 0,
        .enable_velocity = 0,
        .enable_position = 0,
        .enable_motor_output = 0
    },
    [SYSTEM_MODE_FLIGHT] = {
        .enable_rate = 1,
        .enable_angle = 1,
        .enable_velocity = 1,
        .enable_position = 1,
        .enable_motor_output = 1
    },
    [SYSTEM_MODE_TEST] = {
        .enable_rate = 0,
        .enable_angle = 0,
        .enable_velocity = 0,
        .enable_position = 0,
        .enable_motor_output = 1
    }
};
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
