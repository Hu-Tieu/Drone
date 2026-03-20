#include "controller_scheduler_tasks.h"
#include "controller_scheduler.h"
#include "main.h"
#include "IMU/imu.h"
#include "PWM/PWM.h"
#include "RF24/rf24.h"
#include "TOF/tof.h"
#include "Battery/battery.h"
#include <stdio.h>

/* ---------------- Scheduler rate checks ---------------- */
// Ensure slower loops divide evenly into gyro update rate
#if (GYRO_UPDATE_HZ % RATE_LOOP_HZ) != 0U
#error "RATE_LOOP_HZ must divide GYRO_UPDATE_HZ"
#endif
#if (GYRO_UPDATE_HZ % ANGLE_LOOP_HZ) != 0U
#error "ANGLE_LOOP_HZ must divide GYRO_UPDATE_HZ"
#endif
#if (GYRO_UPDATE_HZ % ACCEL_UPDATE_HZ) != 0U
#error "ACCEL_UPDATE_HZ must divide GYRO_UPDATE_HZ"
#endif
#if (GYRO_UPDATE_HZ % MAG_UPDATE_HZ) != 0U
#error "MAG_UPDATE_HZ must divide GYRO_UPDATE_HZ"
#endif

/* ---------------- Task handlers ---------------- */

// Schedule IMU DMA read at gyro rate; include accel/mag only when due
static void handle_gyro_read_req(SchedulerContext_t *ctx) {
    static uint32_t tick = 0U;
    const uint8_t read_accel = ((tick % (GYRO_UPDATE_HZ / ACCEL_UPDATE_HZ)) == 0U);
    const uint8_t read_mag   = ((tick % (GYRO_UPDATE_HZ / MAG_UPDATE_HZ)) == 0U);

    ControlSystem_t *sys = ctx->sys;
    (void)IMU_ReadScheduled(&sys->sensor.imu, read_accel, read_mag);
    tick++;
}

// Runs after gyro DMA completes: estimator update + control loops
static void handle_gyro_update_pipeline(SchedulerContext_t *ctx) {
    static uint32_t count = 0U;
    ControlSystem_t *sys = ctx->sys;
    count++;

    /* =====================================================
     *                   GYRO LPF 800Hz
     * ===================================================== */

     Gyro_LPF(&sys->sensor, &sys->state, &sys->estimator, &sys->gyro_lpf);


    /* =====================================================
     *                   RATE LOOP 400Hz
     * ===================================================== */
    if ((count % (GYRO_UPDATE_HZ / RATE_LOOP_HZ)) == 0U) {

        Controller_Rate_Loop(&sys->rate_ctrl, &sys->state,&sys->signals.torque_pid, &sys->signals.rate_sp);

        Motor_Mixer(&sys->signals.torque_pid, sys->signals.thrust_sp,&sys->signals.motor_desired);

        Motor_Inverse_Model(&sys->signals.motor_desired,&sys->signals.u_esc, sys->sensor.battery);

        SystemMode_t mode =  Receiver_Decode_CH45(&sys->sensor.ibus);

        if (mode == SYSTEM_MODE_SAFETY)     PWM_Motor_Write_All(CCR_MIN, CCR_MIN, CCR_MIN, CCR_MIN);

        else if (mode == SYSTEM_MODE_FLIGHT)  ESC_Output(&sys->signals.u_esc);
        NRF24_Send();
    }

    /* =====================================================
     *                   ACCEL LPF 200Hz
     * ===================================================== */
    if (count % (GYRO_UPDATE_HZ/ACCEL_UPDATE_HZ) == 0) 
    Accel_LPF(&sys->sensor, &sys->state,&sys->estimator, &sys->accel_lpf);
    

    /* =====================================================
     *                   ANGLE LOOP 200Hz
     * ===================================================== */
    if ((count % (GYRO_UPDATE_HZ / ANGLE_LOOP_HZ)) == 0U) {

        // Estimator_UpdateAttitude(&sys->estimator, &sys->state);
    }

    /* =====================================================
     *                   VELOCITY LOOP 100Hz
     * ===================================================== */
    if ((count % (GYRO_UPDATE_HZ / VELOCITY_LOOP_HZ)) == 0U) {
        
    }

    /* =====================================================
    *                   BARO READ + LPF 50Hz
    * ===================================================== */
    if ((count % (GYRO_UPDATE_HZ / BARO_UPDATE_HZ)) == 0U) {
        Read_Barometer(&sys->sensor.baro);

    }

    /* =====================================================
     *                   POSITION LOOP 25Hz
     * ===================================================== */
    if ((count % (GYRO_UPDATE_HZ / POSITION_LOOP_HZ)) == 0U) {

        Mag_LPF(&sys->sensor, &sys->state,&sys->estimator, &sys->mag_lpf);

        if (HAL_GetTick() - rpm_last_capture > 500U) {
            sys->sensor.motor_rpm = 0.0f;
            rpm_last_capture = HAL_GetTick();
        }
    }
}

/* Battery voltage update handlers */
static void handle_vbat_half(SchedulerContext_t *ctx) {
    ADC_GetVbat(&ctx->sys->sensor.battery, ADC_CHANNEL_VBAT_HALF);
}
static void handle_vbat_full(SchedulerContext_t *ctx) {
    ADC_GetVbat(&ctx->sys->sensor.battery, ADC_CHANNEL_VBAT_FULL);
}

/* RF24 IRQ handler */
static void handle_rf24(SchedulerContext_t *ctx) {
    (void)ctx;
    NRF24_IRQHandler();
}

/* Process receiver IBUS data and map to control signals */
static void handle_receiver(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = ctx->sys;
    if (Receiver_ProcessIBUS(&g_receiver, &sys->sensor.ibus)) {
        Receiver_Mapping(&sys->sensor, &sys->signals);
    }
}

/* Update motor RPM measurements */
static void handle_rpm_half(SchedulerContext_t *ctx) {
    Motor_RPM_Update_Half(&ctx->sys->sensor.motor_rpm);
    rpm_last_capture = HAL_GetTick();
}
static void handle_rpm_full(SchedulerContext_t *ctx) {
    Motor_RPM_Update_Full(&ctx->sys->sensor.motor_rpm);
    rpm_last_capture = HAL_GetTick();
}

/* Read TOF sensor if enabled */
static void handle_tof(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = ctx->sys;
    if (tof_enabled) {
        TOF_distance(&tof_last_distance_mm);
        sys->sensor.distance = tof_last_distance_mm;
    }
}

/* Lower-priority placeholder */
static void handle_lower(SchedulerContext_t *ctx) {
    (void)ctx;
}

/* ---------------- Scheduler table ---------------- */
const SchedulerTask_t scheduler_tasks[] = {
    {FLAG_GYRO_UPDATE,       handle_gyro_update_pipeline},
    {FLAG_GYRO_READ_REQ,     handle_gyro_read_req},
    {FLAG_VBAT_HALF_UPDATE,  handle_vbat_half},
    {FLAG_VBAT_FULL_UPDATE,  handle_vbat_full},
    {FLAG_RF24_IRQ,          handle_rf24},
    {FLAG_RECEIVER_IBUS,     handle_receiver},
    {FLAG_RPM_HALF,          handle_rpm_half},
    {FLAG_RPM_FULL,          handle_rpm_full},
    {FLAG_VL53CX_IRQ,        handle_tof},
    {FLAG_LOWER_LOOP,        handle_lower},
};
const int num_scheduler_tasks = sizeof(scheduler_tasks) / sizeof(scheduler_tasks[0]);

/* ---------------- Scheduler runner ---------------- */
void Scheduler_RunTasks(uint32_t *pending_flags, SchedulerContext_t *ctx) {
    for (const SchedulerTask_t *t = scheduler_tasks;
         t < scheduler_tasks + num_scheduler_tasks; ++t) {
        if (*pending_flags & t->flag) {
            *pending_flags &= ~t->flag;
            t->handler(ctx);
        }
    }
}