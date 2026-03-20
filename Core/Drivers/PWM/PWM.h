#ifndef PWM_H
#define PWM_H

#include "stdint.h"

#define RPM_BUFFER_SIZE     16U          // có thể chỉnh sau này
#define RPM_TIMEOUT_MS      300U
#define PULSES_PER_REV      2.0f
#define TIM_PSC             89.0f

#define TIM2_CLOCK 90000000

#define CCR_MIN 1000
#define CCR_MAX 2000

#define RC_CHANNELS 5
#define THROTTLE 2
#define ROLL     3
#define PITCH    1
#define YAW      0
#define SAFETY    4

/* ================= MOTOR ================= */
typedef struct {
    float u;                       // 0..1
    volatile uint32_t *ccr;
} Motor_t;

typedef struct {
    Motor_t FL;
    Motor_t FR;
    Motor_t BL;
    Motor_t BR;
} Motor_System_t;

/* ================= API ================= */
void    PWM_Init(void);
void    PWM_ESC_Calib(void);
void    PWM_Motor_Write_All(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3, uint32_t ccr4);
void    Motor_RPM_Update_Half(float *rpm_out);
void    Motor_RPM_Update_Full(float *rpm_out);

/* Timestamp of last RPM capture (moved out of main.c) */
extern volatile uint32_t rpm_last_capture;

#endif
