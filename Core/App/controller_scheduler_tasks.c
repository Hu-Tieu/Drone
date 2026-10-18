#include "controller_scheduler_tasks.h"
#include "controller_scheduler.h"
#include "main.h"
#include "controller_main.h"
#include "IMU/imu.h"
#include "PWM/PWM.h"
#include "RF24/rf24.h"
#include "TOF/tof.h"
#include "Battery/battery.h"
#include <stdio.h>

/* mode table moved from header to C file to avoid multiple-definition
   and circular-include problems. */
const ModeConfig_t mode_table[] = {
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
    [SYSTEM_MODE_IDLE] = {
        .enable_rate = 1,
        .enable_angle = 1,
        .enable_velocity = 0,
        .enable_position = 0,
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

static SystemMode_t mode = SYSTEM_MODE_SAFETY;
static const ModeConfig_t *cfg = &mode_table[SYSTEM_MODE_SAFETY];
static uint8_t safety_rearm_required = 0U;
static uint8_t safety_rearm_allowed = 1U;
static uint8_t pid_active_last_tick = 0U;
static uint32_t gyro_read_tick = 0U;
static uint32_t gyro_pipeline_count = 0U;

static uint8_t control_pid_enabled(const ControlSystem_t *sys)
{
    float gravity = (sys->sensor.imu.g_est > 0.1f)
        ? sys->sensor.imu.g_est
        : GRAVITY;
    float pid_thrust_threshold = 0.5f * DRONE_MASS * gravity;

    return (uint8_t)((mode != SYSTEM_MODE_TEST) &&
                     (cfg->enable_rate != 0U) &&
                     (sys->signals.thrust_sp >= pid_thrust_threshold));
}

void Scheduler_ForceSafetyMode(uint8_t allow_rearm)
{
    mode = SYSTEM_MODE_SAFETY;
    cfg = &mode_table[SYSTEM_MODE_SAFETY];
    safety_rearm_required = 1U;
    safety_rearm_allowed = allow_rearm;
    pid_active_last_tick = 0U;
    gyro_read_tick = 0U;
    gyro_pipeline_count = 0U;
}

/* ---------------- Task handlers ---------------- */

// Schedule IMU DMA read at gyro rate; include accel/mag only when due
static void handle_gyro_read_req(SchedulerContext_t *ctx) {
    const uint8_t read_accel = ((gyro_read_tick % (GYRO_UPDATE_HZ / ACCEL_UPDATE_HZ)) == 0U);
    const uint8_t read_mag   = ((gyro_read_tick % (GYRO_UPDATE_HZ / MAG_UPDATE_HZ)) == 0U);

    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    (void)IMU_ReadScheduled(&sys->sensor.imu, read_accel, read_mag);
    gyro_read_tick++;
}

// Runs after gyro DMA completes: estimator update + control loops
static void handle_gyro_update_pipeline(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    gyro_pipeline_count++;

    /* =====================================================
     *                   GYRO LPF 800Hz
     * ===================================================== */

     Gyro_LPF(&sys->sensor, &sys->state, &sys->estimator, &sys->gyro_lpf);
     Estimator_Angle(&sys->state, &sys->estimator, GYRO_ANGLE);

    uint8_t pid_active = control_pid_enabled(sys);
    if (pid_active != pid_active_last_tick)
    {
        Rate_PID_Reset(&sys->rate_ctrl);
        Angle_PID_Reset(&sys->angle_ctrl);
        Torque_PID_Reset(&sys->signals.torque_pid);
        pid_active_last_tick = pid_active;
    }

    if (mode == SYSTEM_MODE_SAFETY) {
        Rate_PID_Reset(&sys->rate_ctrl);
        Angle_PID_Reset(&sys->angle_ctrl);
        Torque_PID_Reset(&sys->signals.torque_pid);

        /* Ensure rate setpoints and PID internals are forced to zero
         * so telemetry cannot report stale offsets while in SAFETY mode */
        sys->signals.rate_sp.p = 0.0f;
        sys->signals.rate_sp.q = 0.0f;
        sys->signals.rate_sp.r = 0.0f;

        sys->rate_ctrl.p.integrator = 0.0f;
        sys->rate_ctrl.q.integrator = 0.0f;
        sys->rate_ctrl.r.integrator = 0.0f;

        sys->rate_ctrl.p.prev_error = 0.0f;
        sys->rate_ctrl.q.prev_error = 0.0f;
        sys->rate_ctrl.r.prev_error = 0.0f;

        /* also zero motor commands / ESC outputs for safety */
        ESC_Force(&sys->signals.u_esc);
        ESC_Output(&sys->signals.u_esc);
    }

    /* =====================================================
     *                   RATE LOOP 400Hz
     * ===================================================== */
    if ( (gyro_pipeline_count % (GYRO_UPDATE_HZ / RATE_LOOP_HZ)) == 0U) {
        if (mode == SYSTEM_MODE_TEST)
        {
            float raw_thrust = sys->sensor.ibus.channel[2];
            sys->signals.u_esc.fl = raw_thrust;
            sys->signals.u_esc.fr = raw_thrust;
            sys->signals.u_esc.bl = raw_thrust;
            sys->signals.u_esc.br = raw_thrust;
            ESC_Output(&sys->signals.u_esc);
        }
        else if (control_pid_enabled(sys))
        {
            Controller_Rate_Loop(&sys->rate_ctrl, &sys->state,&sys->signals.torque_pid, &sys->signals.rate_sp);

            Motor_Mixer(&sys->signals.torque_pid, sys->signals.thrust_sp,&sys->signals.motor_desired);

            Motor_Inverse_Model(&sys->signals.motor_desired,&sys->signals.u_esc, sys->sensor.battery);

            ESC_Output(&sys->signals.u_esc);
        }
        else
        {
            Rate_PID_Reset(&sys->rate_ctrl);
            Angle_PID_Reset(&sys->angle_ctrl);
            Torque_PID_Reset(&sys->signals.torque_pid);
            sys->signals.rate_sp.p = 0.0f;
            sys->signals.rate_sp.q = 0.0f;
            sys->signals.rate_sp.r = 0.0f;

            if ((mode == SYSTEM_MODE_IDLE) || (mode == SYSTEM_MODE_FLIGHT))
            {
                Motor_Mixer(&sys->signals.torque_pid,
                            sys->signals.thrust_sp,
                            &sys->signals.motor_desired);
                Motor_Inverse_Model(&sys->signals.motor_desired,
                                    &sys->signals.u_esc,
                                    sys->sensor.battery);
                ESC_Output(&sys->signals.u_esc);
            }
        }
        

        NRF24_Send();
        
    }

    /* =====================================================
     *                   ACCEL LPF 200Hz
     * ===================================================== */
    if (gyro_pipeline_count % (GYRO_UPDATE_HZ/ACCEL_UPDATE_HZ) == 0) {
        Accel_LPF(&sys->sensor, &sys->state,&sys->estimator, &sys->accel_lpf);
        Estimator_Angle(&sys->state, &sys->estimator, ACCEL_ANGLE);
        cfg = &mode_table[mode];
    }

    /* =====================================================
     *                   ANGLE LOOP 200Hz
     * ===================================================== */
    if (cfg->enable_angle &&
        control_pid_enabled(sys) &&
        (gyro_pipeline_count % (GYRO_UPDATE_HZ / ANGLE_LOOP_HZ)) == 0U) {
        Controller_Angle_Loop(&sys->angle_ctrl, &sys->state, &sys->signals.angle_sp, &sys->signals.rate_sp);
    }
    else if (cfg->enable_angle && !control_pid_enabled(sys))
    {
        Angle_PID_Reset(&sys->angle_ctrl);
        sys->signals.rate_sp.p = 0.0f;
        sys->signals.rate_sp.q = 0.0f;
    }

    /* =====================================================
     *                   VELOCITY LOOP 100Hz
     * ===================================================== */
    if (cfg->enable_velocity && (gyro_pipeline_count % (GYRO_UPDATE_HZ / VELOCITY_LOOP_HZ)) == 0U) {
        Convert_BodyToWorld(&sys->state);
    }

    /* =====================================================
    *                   BARO READ + LPF 50Hz
    * ===================================================== */
    if ((gyro_pipeline_count % (GYRO_UPDATE_HZ / BARO_UPDATE_HZ)) == 0U) {
        Read_Barometer(&sys->sensor.baro);
        Barometer_LPF(&sys->sensor, &sys->state, &sys->estimator, &sys->baro_lpf);
    }
     /* =====================================================
     *                   MAG LPF 25Hz
     * ===================================================== */
    if ((gyro_pipeline_count % (GYRO_UPDATE_HZ/ MAG_UPDATE_HZ))== 0U)
    {
        Mag_LPF(&sys->sensor, &sys->state,&sys->estimator, &sys->mag_lpf);
        Estimator_Angle(&sys->state, &sys->estimator, MAG_ANGLE);
        if (HAL_GetTick() - rpm_last_capture > 500U) {
            sys->sensor.motor_rpm = 0.0f;
            rpm_last_capture = HAL_GetTick();
        }
    }
    /* =====================================================
     *                   POSITION LOOP 25Hz
     * ===================================================== */
    if (cfg->enable_position && (gyro_pipeline_count % (GYRO_UPDATE_HZ / POSITION_LOOP_HZ)) == 0U) {

        
    }

    /* Process one calibration sample per gyro cycle after estimator updates. */
    Controller_ProcessResetCalibration(sys);
}

/* Battery voltage update handlers */
static void handle_vbat_half(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    ADC_GetVbat(&sys->sensor.battery, ADC_CHANNEL_VBAT_HALF);
}
static void handle_vbat_full(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    ADC_GetVbat(&sys->sensor.battery, ADC_CHANNEL_VBAT_FULL);
}

/* RF24 IRQ handler */
static void handle_rf24(SchedulerContext_t *ctx) {
    (void)ctx;
    NRF24_IRQHandler();
}

/* Process receiver IBUS data and map to control signals */
static void handle_receiver(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    if (Receiver_ProcessIBUS(&g_receiver, &sys->sensor.ibus)) {
        SystemMode_t requested_mode = Receiver_Decode_CH45(&sys->sensor.ibus);
        if (requested_mode <= SYSTEM_MODE_RESERVED)
            requested_mode = SYSTEM_MODE_SAFETY;

        SystemMode_t previous_mode = mode;
        if (safety_rearm_required != 0U)
        {
            if ((safety_rearm_allowed != 0U) &&
                (requested_mode == SYSTEM_MODE_SAFETY))
                safety_rearm_required = 0U;
            mode = SYSTEM_MODE_SAFETY;
        }
        else
        {
            mode = requested_mode;
        }

        if (mode != previous_mode)
        {
            Rate_PID_Reset(&sys->rate_ctrl);
            Angle_PID_Reset(&sys->angle_ctrl);
            Torque_PID_Reset(&sys->signals.torque_pid);
            sys->signals.rate_sp.p = 0.0f;
            sys->signals.rate_sp.q = 0.0f;
            sys->signals.rate_sp.r = 0.0f;
        }

        cfg = &mode_table[mode];
        Receiver_Mapping(&sys->sensor, &sys->signals, mode);
    }
}

/* Update motor RPM measurements */
static void handle_rpm_half(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    Motor_RPM_Update_Half(&sys->sensor.motor_rpm);
    rpm_last_capture = HAL_GetTick();
}
static void handle_rpm_full(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
    Motor_RPM_Update_Full(&sys->sensor.motor_rpm);
    rpm_last_capture = HAL_GetTick();
}

/* Read TOF sensor if enabled */
static void handle_tof(SchedulerContext_t *ctx) {
    ControlSystem_t *sys = (ControlSystem_t *)ctx->sys;
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