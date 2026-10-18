#include "controller_main.h"
#include "PWM/PWM.h"
#include "controller_scheduler.h"
#include "controller_scheduler_tasks.h"
#include "stdio.h"

#define RESET_CALIB_SAMPLES 100U
#define PID_DERIVATIVE_CUTOFF_HZ 30.0f
#define PID_DERIVATIVE_OUTPUT_LIMIT_FRACTION 0.25f
static inline float clamp(float duty)
{
    return (duty < 0.0f) ? 0.0f : (duty > 1.0f) ? 1.0f : duty;
}

static inline float wrap_pi(float angle)
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
/* ================= SAFETY UTILS ================= */

static inline float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static inline float safe_sqrtf(float x)
{
    if (x <= 0.0f)
        return 0.0f;
    return sqrtf(x);
}

static inline float safe_divf(float num, float den, float den_min)
{
    if (den < den_min)
        den = den_min;
    return num / den;
}
void ESC_Force(MotorCommand_t *u_esc) {
    u_esc->fl = 0.0f;
    u_esc->fr = 0.0f;
    u_esc->bl = 0.0f;
    u_esc->br = 0.0f;
}

/* =========================================================
 *                Convert
 * ========================================================= */
void Receiver_Mapping(DroneSensor_t *sensor, ControlSignals_t *signals, SystemMode_t mode) {
    float g = (sensor->imu.g_est > 0.1f) ? sensor->imu.g_est : GRAVITY;

    if (mode == SYSTEM_MODE_TEST) {
        float raw_thrust = sensor->ibus.channel[2];
        signals->thrust_sp = 0.0f;
        signals->angle_sp.roll = 0.0f;
        signals->angle_sp.pitch = 0.0f;
        signals->angle_sp.yaw = 0.0f;
        signals->rate_sp.p = 0.0f;
        signals->rate_sp.q = 0.0f;
        signals->rate_sp.r = 0.0f;
        Torque_PID_Reset(&signals->torque_pid);
        signals->u_esc.fl = raw_thrust;
        signals->u_esc.fr = raw_thrust;
        signals->u_esc.bl = raw_thrust;
        signals->u_esc.br = raw_thrust;
        return;
    }

    if (mode == SYSTEM_MODE_SAFETY) {
        signals->thrust_sp = 0.0f;
        signals->rate_sp.p = 0.0f;
        signals->rate_sp.q = 0.0f;
        signals->rate_sp.r = 0.0f;
    }
    if (mode == SYSTEM_MODE_IDLE ||
        mode == SYSTEM_MODE_FLIGHT) {
        signals->thrust_sp = sensor->ibus.channel[2] * (DRONE_MASS * g) * 2.0f;
        signals->angle_sp.roll = (sensor->ibus.channel[0]-0.5f) * M_PI/4;   // Roll angle setpoint (rad)
        signals->angle_sp.pitch = -(sensor->ibus.channel[1]-0.5f) * M_PI/4;   // Pitch angle setpoint (rad)
        signals->rate_sp.r = (sensor->ibus.channel[3]-0.5f) * 2 * M_PI/2;   // Yaw angle setpoint (rad)
    }

}

void CF_Init(ComplementaryFilter_t *f, float alpha, float T)
{
    f->alpha = alpha;
    f->dt = T;
    f->angle = 0.0f;
}

static float CF_Predict(ComplementaryFilter_t *f, float gyro_rate)
{
    f->angle += gyro_rate * f->dt;
    return f->angle;
}

static float CF_Correct(ComplementaryFilter_t *f, float acc_angle)
{
    float error = wrap_pi(acc_angle - f->angle);

    f->angle += (1-f->alpha) * error;
    f->angle = wrap_pi(f->angle);

    return f->angle;
}

/* =========================================================
 *                PID INITIALIZATION
 * ========================================================= */
static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                     float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->derivative_initialized = 0U;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

/* =========================================================
 *                    PID UPDATE
 * ========================================================= */
static float PID_Update(PID_t *pid, float setpoint, float measurement, float dt,
                        uint8_t wrap_measurement)
{
    /* =============================
     * 1. Error
     * ============================= */
    float error = setpoint - measurement;

    /* =============================
     * 2. Proportional
     * ============================= */
    float P = pid->kp * error;

    /* =============================
     * 3. Integral (with anti-windup)
     * ============================= */
    pid->integrator += error * dt;

    float I = pid->ki * pid->integrator;

    /* Derivative on measurement avoids a kick when the setpoint changes. */
    if (pid->derivative_initialized == 0U)
    {
        pid->prev_measurement = measurement;
        pid->filtered_derivative = 0.0f;
        pid->derivative_initialized = 1U;
    }
    else
    {
        float measurement_delta = measurement - pid->prev_measurement;
        if (wrap_measurement != 0U)
            measurement_delta = wrap_pi(measurement_delta);

        const float derivative = -measurement_delta / dt;
        const float tau = 1.0f / (2.0f * M_PI * PID_DERIVATIVE_CUTOFF_HZ);
        const float alpha = dt / (tau + dt);

        pid->prev_measurement = measurement;
        pid->filtered_derivative += alpha * (derivative - pid->filtered_derivative);
    }

    float D = pid->kd * pid->filtered_derivative;
    const float d_limit = fminf(fabsf(pid->out_min), fabsf(pid->out_max)) *
                          PID_DERIVATIVE_OUTPUT_LIMIT_FRACTION;
    D = clampf(D, -d_limit, d_limit);

    /* =============================
     * 5. Raw output
     * ============================= */
    float output = P + I + D;

    /* =============================
     * 6. Saturation 
     * ============================= */
    if (output > pid->out_max)
    {
        output = pid->out_max;

        /* Prevent further windup */
        if (error > 0.0f)
            pid->integrator -= error * dt;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;

        if (error < 0.0f)
            pid->integrator -= error * dt;
    }

    /* =============================
     * 7. Store previous error
     * ============================= */
    pid->prev_error = error;

    return output;
}

 /* =========================================================
  *                    PID INTEGRATOR RESET
  * ========================================================= */

static void PID_Reset(PID_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->derivative_initialized = 0U;
}
/* =========================================================
 *                LOW-PASS FILTER INITIALIZATION
 * ========================================================= */
static void LPF_FirstOrder_Init(FirstOrderLPF_t *f, float cutoff_freq, float dt)
{
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
    f->cutoff_freq = cutoff_freq;
    f->alpha = dt / (RC + dt);
    f->dt = dt;
    f->d1 = 0.0f;
}

static float LPF_FirstOrder_Update(FirstOrderLPF_t *f, float input)
{
    f->d1 += f->alpha * (input - f->d1);
    return f->d1;
}

static void LPF_SecondOrder_Init(SecondOrderLPF_t *f, float cutoff_freq, float dt)
{
    float K = tanf(M_PI * cutoff_freq * dt);
    float norm = 1.0f / (1.0f + sqrtf(2.0f) * K + K*K);

    f->b0 = K*K * norm;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;

    f->a1 = 2.0f * (K*K - 1.0f) * norm;
    f->a2 = (1.0f - sqrtf(2.0f) * K + K*K) * norm;

    f->d1 = 0;
    f->d2 = 0;
}

static float LPF_SecondOrder_Update(SecondOrderLPF_t *f, float input)
{
    float output = f->b0 * input + f->d1 ;

    f->d1 = f->b1 * input - f->a1 * output + f->d2;
    f->d2 = f->b2 * input - f->a2 * output ;

    return output;
}


/* =========================================================
 *                CONTROLLER INITIALIZATION
 * ========================================================= */
void Controller_Init(ControlSystem_t *sys)
{
    /* =====================================================
     * 0. Local Shortcuts (readability layer)
     * ===================================================== */
    AttitudeEstimator_t *est   = &sys->estimator;
    AngleController_t   *actrl = &sys->angle_ctrl;
    RateController_t    *rctrl = &sys->rate_ctrl;
    FeedForward_t       *ff    = &sys->feedforward;
    ControlSignals_t    *sig   = &sys->signals;
    Inertia_t           *J     = &sys->inertia;

    /* =====================================================
     * 1. ESTIMATOR LAYER
     * ===================================================== */
    const float gyro_dt    = 1.0f/800.0f;   // 800 Hz
    const float accele_dt  = 1.0f/200.0f;    // 200 Hz 
    const float mag_dt     = 1.0f/25.0f;     // 25 Hz 
    const float baro_dt    = 1.0f/50.0f;     // 50 Hz
    const float alpha = 0.98f; 

    CF_Init(&est->phi,  alpha, gyro_dt);
    CF_Init(&est->theta, alpha, gyro_dt);
    CF_Init(&est->psi,   alpha, gyro_dt);

    LPF_FirstOrder_Init(&sys->accel_lpf.ax, 50.0f, accele_dt);
    LPF_FirstOrder_Init(&sys->accel_lpf.ay, 50.0f, accele_dt);
    LPF_FirstOrder_Init(&sys->accel_lpf.az, 50.0f, accele_dt);

    LPF_SecondOrder_Init(&sys->gyro_lpf.p, 80.0f, gyro_dt);
    LPF_SecondOrder_Init(&sys->gyro_lpf.q, 80.0f, gyro_dt);
    LPF_SecondOrder_Init(&sys->gyro_lpf.r, 80.0f, gyro_dt);

    LPF_FirstOrder_Init(&sys->mag_lpf.mx, 10.0f, mag_dt);
    LPF_FirstOrder_Init(&sys->mag_lpf.my, 10.0f, mag_dt);
    LPF_FirstOrder_Init(&sys->mag_lpf.mz, 10.0f, mag_dt);

    LPF_FirstOrder_Init(&sys->baro_lpf, 10.0f, baro_dt);

    Read_Magnetic_Blocking(&sys->sensor.imu);
    sys->estimator.psi.angle = -atan2f(sys->sensor.imu.my, sys->sensor.imu.mx);
    sys->state.euler.z = sys->estimator.psi.angle;

    /* =====================================================
     * 2. ANGLE CONTROLLER (Outer Loop)
     * ===================================================== */
    PID_Init(&actrl->roll,  ROLL_KP, ROLL_KI, ROLL_KD,  -30.0f, 30.0f);
    PID_Init(&actrl->pitch, PITCH_KP, PITCH_KI, PITCH_KD,  -30.0f, 30.0f);
    PID_Init(&actrl->yaw,   YAW_KP, YAW_KI, YAW_KD, -30.0f, 30.0f);

    /* =====================================================
     * 3. RATE CONTROLLER (Inner Loop)
     * ===================================================== */
    PID_Init(&rctrl->p, P_KP,  P_KI,  P_KD,  -10.0f, 10.0f);
    PID_Init(&rctrl->q, Q_KP,  Q_KI,  Q_KD,  -10.0f, 10.0f);
    PID_Init(&rctrl->r, R_KP,  R_KI, R_KD, -10.0f, 10.0f);

    /* =====================================================
     * 4. INERTIA MODEL (Physical parameters)
     * ===================================================== */
    J->Jx = Jxx;
    J->Jy = Jyy;
    J->Jz = Jzz;

    /* =====================================================
     * 5. FEEDFORWARD TORQUE
     * ===================================================== */
    ff->tau_x = 0.0f;
    ff->tau_y = 0.0f;
    ff->tau_z = 0.0f;

    /* =====================================================
     * 6. CONTROL PIPELINE RESET
     * ===================================================== */

    /* --- Angle setpoints --- */
    sig->angle_sp.roll  = 0.0f;
    sig->angle_sp.pitch = 0.0f;
    sig->angle_sp.yaw   = 0.0f;

    /* --- Rate setpoints --- */
    sig->rate_sp.p = 0.0f;
    sig->rate_sp.q = 0.0f;
    sig->rate_sp.r = 0.0f;

    /* --- Torque values --- */
    sig->torque_pid.tau_x   = 0.0f;
    sig->torque_pid.tau_y   = 0.0f;
    sig->torque_pid.tau_z   = 0.0f;

    sig->torque_ff.tau_x    = 0.0f;
    sig->torque_ff.tau_y    = 0.0f;
    sig->torque_ff.tau_z    = 0.0f;

    sig->torque_total.tau_x = 0.0f;
    sig->torque_total.tau_y = 0.0f;
    sig->torque_total.tau_z = 0.0f;

    /* --- Thrust --- */
    sig->thrust_sp = 0.0f;

    /* --- Motor commands --- */
    sig->motor_desired.fl = 0.0f;
    sig->motor_desired.fr = 0.0f;
    sig->motor_desired.bl = 0.0f;
    sig->motor_desired.br = 0.0f;

    sig->u_esc.fl = 0.0f;
    sig->u_esc.fr = 0.0f;
    sig->u_esc.bl = 0.0f;
    sig->u_esc.br = 0.0f;

    /* --- Reset/Calibration state --- */
    sys->state.euler_offset.x = 0.0f;
    sys->state.euler_offset.y = 0.0f;
    sys->state.euler_offset.z = 0.0f;

    sys->state.rate_offset.x = 0.0f;
    sys->state.rate_offset.y = 0.0f;
    sys->state.rate_offset.z = 0.0f;

    sys->reset_calib.angle_active = 0U;
    sys->reset_calib.rate_active = 0U;
    sys->reset_calib.angle_count = 0U;
    sys->reset_calib.rate_count = 0U;
    sys->reset_calib.angle_sum_x = 0.0f;
    sys->reset_calib.angle_sum_y = 0.0f;
    sys->reset_calib.rate_sum_p = 0.0f;
    sys->reset_calib.rate_sum_q = 0.0f;
    sys->reset_calib.rate_sum_r = 0.0f;
}

void Rate_PID_Reset(RateController_t *rctrl)
{
    PID_Reset(&rctrl->p);
    PID_Reset(&rctrl->q);
    PID_Reset(&rctrl->r);
}

/* =========================================================
 *              TORQUE RESET (Output Reset)
 * ========================================================= */
void Torque_PID_Reset(Torque_t *torque_pid)
{
    torque_pid->tau_x = 0.0f;
    torque_pid->tau_y = 0.0f;
    torque_pid->tau_z = 0.0f;
}

void Angle_PID_Reset(AngleController_t *actrl)
{
    PID_Reset(&actrl->roll);
    PID_Reset(&actrl->pitch);
    PID_Reset(&actrl->yaw);
}

void Controller_RequestAngleReset(ControlSystem_t *sys)
{
    if (sys == NULL)
    {
        return;
    }

    /* Ignore duplicate reset requests while sampling is in progress. */
    if (sys->reset_calib.angle_active)
    {
        return;
    }

    sys->reset_calib.angle_active = 1U;
    sys->reset_calib.angle_count = 0U;
    sys->reset_calib.angle_sum_x = 0.0f;
    sys->reset_calib.angle_sum_y = 0.0f;

    /* Start calibration processing immediately; the scheduler will keep
     * feeding samples on subsequent gyro updates. */
    Controller_ProcessResetCalibration(sys);
}

void Controller_RequestRateReset(ControlSystem_t *sys)
{
    if (sys == NULL)
    {
        return;
    }

    /* Ignore duplicate reset requests while sampling is in progress. */
    if (sys->reset_calib.rate_active)
    {
        return;
    }

    sys->reset_calib.rate_active = 1U;
    sys->reset_calib.rate_count = 0U;
    sys->reset_calib.rate_sum_p = 0.0f;
    sys->reset_calib.rate_sum_q = 0.0f;
    sys->reset_calib.rate_sum_r = 0.0f;

    /* Start calibration processing immediately; the scheduler will keep
     * feeding samples on subsequent gyro updates. */
    Controller_ProcessResetCalibration(sys);
}

void Controller_ProcessResetCalibration(ControlSystem_t *sys)
{
    if (sys == NULL)
    {
        return;
    }

    if (sys->reset_calib.angle_active)
    {
        sys->reset_calib.angle_sum_x += sys->state.euler.x;
        sys->reset_calib.angle_sum_y += sys->state.euler.y;
        sys->reset_calib.angle_count++;

        if (sys->reset_calib.angle_count >= RESET_CALIB_SAMPLES)
        {
            float avg_x = sys->reset_calib.angle_sum_x / (float)RESET_CALIB_SAMPLES;
            float avg_y = sys->reset_calib.angle_sum_y / (float)RESET_CALIB_SAMPLES;

            /* Incremental zeroing: keep old offset, remove current mean */
            sys->state.euler_offset.x -= avg_x;
            sys->state.euler_offset.y -= avg_y;

            sys->reset_calib.angle_active = 0U;
            sys->reset_calib.angle_count = 0U;
            sys->reset_calib.angle_sum_x = 0.0f;
            sys->reset_calib.angle_sum_y = 0.0f;
        }
    }

    if (sys->reset_calib.rate_active)
    {
        sys->reset_calib.rate_sum_p += sys->state.omega_body.x;
        sys->reset_calib.rate_sum_q += sys->state.omega_body.y;
        sys->reset_calib.rate_sum_r += sys->state.omega_body.z;
        sys->reset_calib.rate_count++;

        if (sys->reset_calib.rate_count >= RESET_CALIB_SAMPLES)
        {
            float avg_p = sys->reset_calib.rate_sum_p / (float)RESET_CALIB_SAMPLES;
            float avg_q = sys->reset_calib.rate_sum_q / (float)RESET_CALIB_SAMPLES;
            float avg_r = sys->reset_calib.rate_sum_r / (float)RESET_CALIB_SAMPLES;

            /* omega_body = gyro_filtered - rate_offset */
            sys->state.rate_offset.x += avg_p;
            sys->state.rate_offset.y += avg_q;
            sys->state.rate_offset.z += avg_r;

            sys->reset_calib.rate_active = 0U;
            sys->reset_calib.rate_count = 0U;
            sys->reset_calib.rate_sum_p = 0.0f;
            sys->reset_calib.rate_sum_q = 0.0f;
            sys->reset_calib.rate_sum_r = 0.0f;
        }
    }
}

void Estimator_Angle(DroneState_t *state, AttitudeEstimator_t *estimator,uint8_t TYPE)
{
    /* =====================================================
     * 0. ROLL & PITCH FROM GYRO
     * ===================================================== */
    if (TYPE == GYRO_ANGLE)
    {
        float p = state->omega_body.x;
        float q = state->omega_body.y;
        float r = state->omega_body.z;

        /* Use raw estimator states here (before display/control offsets)
         * to avoid feeding offset back into kinematic prediction. */
        float phi   = estimator->phi.angle;
        float theta = estimator->theta.angle;

        float phi_dot   = p + q*sinf(phi)*tanf(theta) + r*cosf(phi)*tanf(theta);
        float theta_dot = q*cosf(phi) - r*sinf(phi);
        float psi_dot   = (q*sinf(phi) + r*cosf(phi)) / cosf(theta);

        state->euler.x = CF_Predict(&estimator->phi, phi_dot);
        state->euler.y = CF_Predict(&estimator->theta, theta_dot);
        state->euler.z = wrap_pi(CF_Predict(&estimator->psi, psi_dot));

        state->euler.x = wrap_pi(state->euler.x + state->euler_offset.x);
        state->euler.y = wrap_pi(state->euler.y + state->euler_offset.y);

        return;
    }
    
    else if (TYPE == ACCEL_ANGLE)
    {
        float ax = state->accel_body.x;
        float ay = state->accel_body.y;
        float az = state->accel_body.z;

        float phi  = CF_Correct(&estimator->phi, atan2f(ay, az));
        float theta = CF_Correct(&estimator->theta, atan2f(-ax, sqrtf(ay*ay + az*az)));

        state->euler.x = wrap_pi(phi + state->euler_offset.x);
        state->euler.y = wrap_pi(theta + state->euler_offset.y);
        return;
    }
    else if (TYPE == MAG_ANGLE)
    {
        float mx = state->mag.x;
        float my = state->mag.y;
        float mz = state->mag.z;

        float phi  = state->euler.x;
        float theta = state->euler.y;

        float mx_h = mx * cosf(theta) + mz * sinf(theta);
        float my_h = mx * sinf(phi) * sinf(theta)
                    + my * cosf(phi)
                    - mz * sinf(phi) * cosf(theta);

        float psi_mag = -atan2f(my_h, mx_h);

        state->euler.z = CF_Correct(&estimator->psi, psi_mag);
    }

}
void Convert_BodyToWorld(DroneState_t *state)
{
    /* =====================================================
     * 1. ROTATION MATRIX (Body → World)
     * ===================================================== */

    float phi   = state->euler.x;
    float theta = state->euler.y;
    float gamma = state->euler.z;

    float cphi = cosf(phi);
    float sphi = sinf(phi);

    float stheta = sinf(theta);

    float cgamma = cosf(gamma);
    float sgamma = sinf(gamma);

    float R[9];

    R[0] = cosf(theta) * cosf(gamma);
    R[1] = sphi * stheta * cgamma - cphi * sgamma;
    R[2] = cphi * stheta * cgamma + sphi * sgamma;

    R[3] = cosf(theta) * sgamma;
    R[4] = sphi * stheta * sgamma + cphi * cgamma;
    R[5] = cphi * stheta * sgamma - sphi * cgamma;

    R[6] = -stheta;
    R[7] = sphi * cosf(theta);
    R[8] = cphi * cosf(theta);

    /* =====================================================
     * 2. WORLD ACCELERATION
     * ===================================================== */

    float ax = state->accel_body.x;
    float ay = state->accel_body.y;
    float az = state->accel_body.z;

    float ax_w = R[0]*ax + R[1]*ay + R[2]*az;
    float ay_w = R[3]*ax + R[4]*ay + R[5]*az;
    float az_w = R[6]*ax + R[7]*ay + R[8]*az;

    // NED frame (+Z down)
    /* gravity compensation: use estimated gravity in state when available */
    float g_comp = (state->gravity_est > 0.1f) ? state->gravity_est : GRAVITY;
    az_w -= g_comp;  // GRAVITY COMPENSATION

    state->accel_world.x = ax_w;
    state->accel_world.y = ay_w;
    state->accel_world.z = az_w;
}

void Gyro_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator,GyroLPF_t *gyro_lpf)
{
    /* =====================================================
     * 1. RAW GYRO
     * ===================================================== */
    
    float gx_raw = sensor->imu.gx;
    float gy_raw = sensor->imu.gy;
    float gz_raw = sensor->imu.gz;

    /* =====================================================
     * 2. LOW-PASS FILTER
     * ===================================================== */
    float gx_f = LPF_SecondOrder_Update(&gyro_lpf->p, gx_raw);
    float gy_f = LPF_SecondOrder_Update(&gyro_lpf->q, gy_raw);
    float gz_f = LPF_SecondOrder_Update(&gyro_lpf->r, gz_raw);

    state->omega_body.x = gx_f - state->rate_offset.x;
    state->omega_body.y = gy_f - state->rate_offset.y;
    state->omega_body.z = gz_f - state->rate_offset.z;
}

void Accel_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, AccelLPF_t *accel_lpf)
{
    /* =====================================================
     * 1. RAW ACCEL
     * ===================================================== */
    
     float ax = sensor->imu.ax;
     float ay = sensor->imu.ay;
     float az = sensor->imu.az;
    /* =====================================================
     * 2. LOW-PASS FILTER
     * ===================================================== */
    ax = LPF_FirstOrder_Update(&accel_lpf->ax, ax);
    ay = LPF_FirstOrder_Update(&accel_lpf->ay, ay);
    az = LPF_FirstOrder_Update(&accel_lpf->az, az);

    state->accel_body.x = ax;
    state->accel_body.y = ay;
    state->accel_body.z = az;

    /* propagate calibrated gravity estimate from sensor into state */
    state->gravity_est = (sensor->imu.g_est > 0.1f) ? sensor->imu.g_est : GRAVITY;

}

void Mag_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, MagLPF_t *mag_lpf)
{
    /* =====================================================
     * 1. RAW MAG
     * ===================================================== */

    float mx = sensor->imu.mx;
    float my = sensor->imu.my;
    float mz = sensor->imu.mz;

    /* =====================================================
     * 2. LOW-PASS FILTER
     * ===================================================== */
    mx = LPF_FirstOrder_Update(&mag_lpf->mx, mx);
    my = LPF_FirstOrder_Update(&mag_lpf->my, my);
    mz = LPF_FirstOrder_Update(&mag_lpf->mz, mz);

    state->mag.x = mx;
    state->mag.y = my;
    state->mag.z = mz;
}
void Barometer_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, FirstOrderLPF_t *baro_lpf)
{
    float alt_raw = sensor->baro.pressure;
    float alt_f = LPF_FirstOrder_Update(baro_lpf, alt_raw);
    state->baro_altitude = alt_f;
}

void Motor_Mixer(Torque_t *torque_total,
                 float thrust_sp,
                 MotorCommand_t *motor_desired)
{
    float arm_projection = ARM_LENGTH / sqrtf(2.0f);

    /* Base thrust → omega^2 */
    float T  = thrust_sp / (4.0f * KF);

    /* Torque → omega^2 contribution */
    float Tx = torque_total->tau_x / (4.0f * arm_projection * KF);
    float Ty = torque_total->tau_y / (4.0f * arm_projection * KF);
    float Tz = torque_total->tau_z / (4.0f * KM);

    /* Compute omega^2 */
    float w2_fl = T + Tx + Ty + Tz;
    float w2_fr = T - Tx + Ty - Tz;
    float w2_bl = T + Tx - Ty - Tz;
    float w2_br = T - Tx - Ty + Tz;

    /* Safe sqrt */
    float w_fl = safe_sqrtf(w2_fl);
    float w_fr = safe_sqrtf(w2_fr);
    float w_bl = safe_sqrtf(w2_bl);
    float w_br = safe_sqrtf(w2_br);

    /* Saturation */
    motor_desired->fl = clampf(w_fl, W_MIN, W_MAX);
    motor_desired->fr = clampf(w_fr, W_MIN, W_MAX);
    motor_desired->bl = clampf(w_bl, W_MIN, W_MAX);
    motor_desired->br = clampf(w_br, W_MIN, W_MAX);
}

void Motor_Inverse_Model(MotorCommand_t *motor_desired,
                         MotorCommand_t *u_esc,
                         float Vbat)
{


    float u_fl = (A_COEFF * motor_desired->fl * motor_desired->fl
                 + B_COEFF * motor_desired->fl);

    float u_fr = (A_COEFF * motor_desired->fr * motor_desired->fr
                 + B_COEFF * motor_desired->fr);

    float u_bl = (A_COEFF * motor_desired->bl * motor_desired->bl
                 + B_COEFF * motor_desired->bl);

    float u_br = (A_COEFF * motor_desired->br * motor_desired->br
                 + B_COEFF * motor_desired->br);

    /* Safe division */
    u_esc->fl = safe_divf(u_fl, Vbat, VBAT_MIN);
    u_esc->fr = safe_divf(u_fr, Vbat, VBAT_MIN);
    u_esc->bl = safe_divf(u_bl, Vbat, VBAT_MIN);
    u_esc->br = safe_divf(u_br, Vbat, VBAT_MIN);

}

void ESC_Output(MotorCommand_t *u_esc) {

    // Clamp duty về [0,1]
    // Bảo vệ ESC và tránh overflow PWM
    float u_fl = clamp(u_esc->fl);
    float u_fr = clamp(u_esc->fr);
    float u_bl = clamp(u_esc->bl);
    float u_br = clamp(u_esc->br);

    // Mapping duty (0-1) sang giá trị CCR PWM
    uint16_t ccr_fl = (uint16_t)(u_fl * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_fr = (uint16_t)(u_fr * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_bl = (uint16_t)(u_bl * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_br = (uint16_t)(u_br * (CCR_MAX - CCR_MIN) + CCR_MIN);

    // Xuất PWM cho 4 motor
    PWM_Motor_Write_All(ccr_fl, ccr_fr, ccr_bl, ccr_br);
}

void Controller_Rate_Loop(RateController_t *rctrl, DroneState_t *state, Torque_t *torque_pid, RateSetpoint_t *rate_sp) {
    
    torque_pid->tau_x = PID_Update(&rctrl->p, rate_sp->p, state->omega_body.x,
                                   1.0f / RATE_LOOP_HZ, 0U);
    torque_pid->tau_y = PID_Update(&rctrl->q, rate_sp->q, state->omega_body.y,
                                   1.0f / RATE_LOOP_HZ, 0U);
    torque_pid->tau_z = PID_Update(&rctrl->r, rate_sp->r, state->omega_body.z,
                                   1.0f / RATE_LOOP_HZ, 0U);

}

void Controller_Angle_Loop(AngleController_t *actrl,DroneState_t *state,
                    AngleSetpoint_t *angle_sp,RateSetpoint_t *rate_sp){
    float dt = 1.0f / ANGLE_LOOP_HZ;

    float err_roll  = angle_sp->roll  - state->euler.x;
    float err_pitch = angle_sp->pitch - state->euler.y;

    const float DB = 0.0f*M_PI/180.0f;  // Deadband of 0.3 degrees in radians

    // ===== ROLL =====
    if (fabsf(err_roll) > DB)
    {
        rate_sp->p = PID_Update(&actrl->roll, angle_sp->roll, state->euler.x,
                                dt, 1U);
    }
    else
    {
        rate_sp->p = 0.0f;
        actrl->roll.integrator = 0.0f;
    }

    // ===== PITCH =====
    if (fabsf(err_pitch) > DB)
    {
        rate_sp->q = PID_Update(&actrl->pitch, angle_sp->pitch, state->euler.y,
                                dt, 1U);
    }
    else
    {
        rate_sp->q = 0.0f;
        actrl->pitch.integrator = 0.0f;
    }
}