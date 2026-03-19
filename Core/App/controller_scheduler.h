#ifndef CONTROLLER_SCHEDULER_H
#define CONTROLLER_SCHEDULER_H

#include "stdint.h"

/* ============================================================
 *                  SYSTEM BASE TIMING
 * ============================================================ */
#define SCHEDULER_BASE_HZ      1600U      // TIM6 = 1.6kHz master tick


/* ============================================================
 *                  CONTROL LOOP RATES
 *      (Chỉ chỉnh tần số ở đây)
 * ============================================================ */
#define GYRO_UPDATE_HZ          800U
#define RATE_LOOP_HZ            400U
#define ANGLE_LOOP_HZ           200U
#define VELOCITY_LOOP_HZ        100U
#define POSITION_LOOP_HZ        25U
#define LOWER_LOOP_HZ            10U
#define TOF_POLL_HZ              20U
/* ============================================================
 *                  AUTO DIVIDER CALCULATION
 * ============================================================ */
#define DIV_GYRO_UPDATE          (SCHEDULER_BASE_HZ / GYRO_UPDATE_HZ)
#define DIV_RATE_LOOP            (SCHEDULER_BASE_HZ / RATE_LOOP_HZ)
#define DIV_ANGLE_LOOP           (SCHEDULER_BASE_HZ / ANGLE_LOOP_HZ)
#define DIV_VELOCITY_LOOP        (SCHEDULER_BASE_HZ / VELOCITY_LOOP_HZ)
#define DIV_POSITION_LOOP        (SCHEDULER_BASE_HZ / POSITION_LOOP_HZ)
#define DIV_LOWER_LOOP           (SCHEDULER_BASE_HZ / LOWER_LOOP_HZ)
#define DIV_TOF_POLL             (SCHEDULER_BASE_HZ / TOF_POLL_HZ)
/* ============================================================
 *                  SYSTEM FLAGS
 * ============================================================ */
typedef uint32_t system_flag_t;

#define FLAG_RATE_LOOP          (1U << 0)
#define FLAG_ANGLE_LOOP         (1U << 1)
#define FLAG_VELOCITY_LOOP      (1U << 2)
#define FLAG_POSITION_LOOP      (1U << 3)
#define FLAG_LOWER_LOOP         (1U << 4)
#define FLAG_RPM_HALF           (1U << 5)
#define FLAG_RPM_FULL           (1U << 6)
#define FLAG_CALIB              (1U << 7)
#define FLAG_FLIGHT             (1U << 8)    
#define FLAG_RF24_IRQ           (1U << 9)
#define FLAG_VL53CX_IRQ         (1U << 10)
#define FLAG_VBAT_HALF_UPDATE   (1U << 11)
#define FLAG_VBAT_FULL_UPDATE   (1U << 12)
#define FLAG_GYRO_UPDATE        (1U << 13)
typedef struct
{
    system_flag_t system;
} flag_t;


extern volatile flag_t g_flag;

/* Return one ready flag with highest priority. Returns 0 when no task is ready. */
system_flag_t Scheduler_SelectHighestPriority(system_flag_t ready_flags);

/* Called in main loop when a flagged task is about to run. */
void Scheduler_ReportTaskDelayAndClear(system_flag_t flag);

/* Set a scheduler flag from ISR/callback context. */
void Scheduler_SetFlagFromISR(system_flag_t flag);

/* Record execution time (in DWT cycles) for a task instance. */
void Scheduler_RecordTaskExecCycles(system_flag_t flag, uint32_t exec_cycles);

/* Get the most recent inter-set interval (in DWT cycles) for a flag. */
uint32_t Scheduler_GetLastIntervalCycles(system_flag_t flag);

/* Get the worst inter-set interval (in DWT cycles) observed since last reset. */
uint32_t Scheduler_GetWorstIntervalCycles(system_flag_t flag);

/* Latency: time from flag set -> when it was cleared (in microseconds).
 * Returns the most recent latency (no clear).
 */
uint32_t Scheduler_GetLastLatencyUs(system_flag_t flag);

/* Pop the worst latency (us) observed since last pop (clears stored worst). */
uint32_t Scheduler_PopWorstLatencyUs(system_flag_t flag);

/* Pop counters accumulated since last pop: miss count and error count. */
uint32_t Scheduler_PopMissCount(system_flag_t flag);
uint32_t Scheduler_PopErrorCount(system_flag_t flag);

#endif