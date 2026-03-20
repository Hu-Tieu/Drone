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
#define ACCEL_UPDATE_HZ         200U
#define BARO_UPDATE_HZ          50U
#define MAG_UPDATE_HZ           25U
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
#define DIV_ACCEL_UPDATE         (SCHEDULER_BASE_HZ / ACCEL_UPDATE_HZ)
#define DIV_MAG_UPDATE           (SCHEDULER_BASE_HZ / MAG_UPDATE_HZ)
#define DIV_BARO_UPDATE          (SCHEDULER_BASE_HZ / BARO_UPDATE_HZ)
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
#define FLAG_RF24_IRQ           (1U << 7)
#define FLAG_VL53CX_IRQ         (1U << 8)
#define FLAG_VBAT_HALF_UPDATE   (1U << 9)
#define FLAG_VBAT_FULL_UPDATE   (1U << 10)
#define FLAG_GYRO_UPDATE        (1U << 11)
/* Receiver IBUS DMA complete flag */
#define FLAG_RECEIVER_IBUS      (1U << 12)
/* Request a new gyro DMA frame (gyro + optional accel/mag). */
#define FLAG_GYRO_READ_REQ      (1U << 13)
typedef struct
{
    system_flag_t system;
} flag_t;


extern volatile flag_t g_flag;

/* Runtime phase-shift configuration (microseconds) for non-rate loops.
 * Effective resolution is one scheduler tick: 1e6 / SCHEDULER_BASE_HZ (625us at 1600Hz).
 */
extern volatile uint32_t g_phase_angle_us;
extern volatile uint32_t g_phase_velocity_us;
extern volatile uint32_t g_phase_position_us;
extern volatile uint32_t g_phase_lower_us;

/* Return one ready flag with highest priority. Returns 0 when no task is ready. */
/* Scheduler_SelectHighestPriority removed (unused) */

/* Set a scheduler flag from ISR/callback context. */
void Scheduler_SetFlagFromISR(system_flag_t flag);


#endif