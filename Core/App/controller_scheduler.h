#ifndef CONTROLLER_SCHEDULER_H
#define CONTROLLER_SCHEDULER_H

#include "stdint.h"

/* ============================================================
 *                  SYSTEM BASE TIMING
 * ============================================================ */
#define SCHEDULER_BASE_HZ      1000U      // TIM6 = 1kHz master tick


/* ============================================================
 *                  CONTROL LOOP RATES
 *      (Chỉ chỉnh tần số ở đây)
 * ============================================================ */
#define RATE_LOOP_HZ           1000U
#define ANGLE_LOOP_HZ           500U
#define VELOCITY_LOOP_HZ        200U
#define POSITION_LOOP_HZ        50U
#define LOWER_LOOP_HZ            10U
/* ============================================================
 *                  AUTO DIVIDER CALCULATION
 * ============================================================ */
#define DIV_RATE_LOOP            (SCHEDULER_BASE_HZ / RATE_LOOP_HZ)
#define DIV_ANGLE_LOOP           (SCHEDULER_BASE_HZ / ANGLE_LOOP_HZ)
#define DIV_VELOCITY_LOOP        (SCHEDULER_BASE_HZ / VELOCITY_LOOP_HZ)
#define DIV_POSITION_LOOP        (SCHEDULER_BASE_HZ / POSITION_LOOP_HZ)
#define DIV_LOWER_LOOP           (SCHEDULER_BASE_HZ / LOWER_LOOP_HZ)
/* ============================================================
 *                  SYSTEM FLAGS
 * ============================================================ */
typedef uint32_t system_flag_t;

#define FLAG_RATE_LOOP          (1U << 0)
#define FLAG_ANGLE_LOOP         (1U << 1)
#define FLAG_VELOCITY_LOOP      (1U << 2)
#define FLAG_POSITION_LOOP      (1U << 3)
#define FLAG_LOWER_LOOP         (1U << 7)

#define FLAG_CALIB              (1U << 4)
#define FLAG_RPM_HALF           (1U << 5)
#define FLAG_RPM_FULL           (1U << 6)
#define FLAG_FLIGHT             (1U << 8)    
#define FLAG_RF24_IRQ           (1U << 9)
#define FLAG_VL53CX_IRQ         (1U << 10)
#define FLAG_VBAT_HALF_UPDATE   (1U << 11)
#define FLAG_VBAT_FULL_UPDATE   (1U << 12)

typedef struct
{
    system_flag_t system;
} flag_t;


extern volatile flag_t g_flag;
#endif