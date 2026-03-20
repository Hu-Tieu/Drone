/*
 * PWM.c - Motor Control and RC Receiver Signal Processing
 *
 * Created on: Feb 13, 2026
 * Author: trinh
 *
 * Functions:
 *   - PWM_Init: Initialize timers and start IC/PWM capture
 *   - RC_Update_Command: Convert RC pulse to normalized commands
 *   - MOTOR_Update: Mix control commands to motor outputs
 *   - RPM_Process: Calculate RPM from encoder pulses
 */

#include "PWM.h"
#include "tim.h"
static Motor_System_t motor;
static uint32_t rpm_buffer[RPM_BUFFER_SIZE];
/* Last main-loop timestamp where RPM was captured (ms). */
volatile uint32_t rpm_last_capture = 0U;

/* ================= HELPER FUNCTIONS ================= */

static inline void Motor_Write(Motor_t *m, uint32_t value)
{
    *(m->ccr) = value;
}

/* ================= INIT ================= */
void PWM_Init(void)
{    /* Start Motor PWM (TIM2) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    /* Map CCR registers */
    motor.FL.ccr = &TIM2->CCR1;
    motor.FR.ccr = &TIM2->CCR4;
    motor.BL.ccr = &TIM2->CCR3;
    motor.BR.ccr = &TIM2->CCR2;

    /* Start Encoder DMA (TIM5) */
    HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_2, rpm_buffer, RPM_BUFFER_SIZE);
    PWM_Motor_Write_All(CCR_MIN, CCR_MIN, CCR_MIN, CCR_MIN);
}

/* ================= RC COMMAND ================= */
void PWM_ESC_Calib(void)
{   

    /* Set to max throttle for 3 seconds */
    PWM_Motor_Write_All(CCR_MAX, CCR_MAX, CCR_MAX, CCR_MAX);
    HAL_Delay(3000);

    /* Set to min throttle for 6 seconds */
    PWM_Motor_Write_All(CCR_MIN, CCR_MIN, CCR_MIN, CCR_MIN);
    HAL_Delay(6000);
}

void PWM_Motor_Write_All(uint32_t FL_Motor, uint32_t FR_Motor, uint32_t BL_Motor, uint32_t BR_Motor)
{
    Motor_Write(&motor.FL, FL_Motor);
    Motor_Write(&motor.FR, FR_Motor);
    Motor_Write(&motor.BL, BL_Motor);
    Motor_Write(&motor.BR, BR_Motor);
}

/* ================= RPM PROCESS ================= */
static float RPM_Compute(uint32_t *buf, uint32_t length)
{
    if (length < 2U)
        return 0.0f;

    uint64_t total_ticks = 0;

    for (uint32_t i = 0; i < (length - 1U); i++)
    {
        uint32_t diff;

        if (buf[i + 1] >= buf[i])
            diff = buf[i + 1] - buf[i];
        else
            diff = (0xFFFFFFFFU - buf[i]) + buf[i + 1] + 1U;

        total_ticks += diff;
    }

    if (total_ticks == 0)
        return 0.0f;

    float timer_freq = (float)TIM2_CLOCK / (TIM_PSC + 1.0f);

    return (timer_freq * 60.0f * (length - 1U)) /
        ((float)total_ticks*2.0f); /* 2 pulses per revolution */
}

void Motor_RPM_Update_Half(float *rpm_out)
{
    *rpm_out = RPM_Compute(&rpm_buffer[0], RPM_BUFFER_SIZE / 2);
}

void Motor_RPM_Update_Full(float *rpm_out)
{
    *rpm_out = RPM_Compute(&rpm_buffer[RPM_BUFFER_SIZE / 2], RPM_BUFFER_SIZE / 2);
}