#include "controller_main.h"
#include "PWM/PWM.h"
#include "controller_scheduler.h"
#include "stdio.h"
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
    u_esc->w_fl = 0.0f;
    u_esc->w_fr = 0.0f;
    u_esc->w_bl = 0.0f;
    u_esc->w_br = 0.0f;
}

/* =========================================================
 *                Convert
 * ========================================================= */
void Receiver_Mapping(DroneSensor_t *sensor, ControlSignals_t *signals) {

    signals->thrust_sp = sensor->ibus.channel[2] * (DRONE_MASS * GRAVITY )*1.5f;  
    signals->rate_sp.p = (sensor->ibus.channel[0]-0.5f) * 2 * M_PI/2;   // Roll rate setpoint (rad/s)
    signals->rate_sp.q = (sensor->ibus.channel[1]-0.5f) * 2 * M_PI/2;   // Pitch rate setpoint (rad/s)
    signals->rate_sp.r = (sensor->ibus.channel[3]-0.5f) * 2 * M_PI/2;   // Yaw rate setpoint (rad/s)

}

void CF_Init(ComplementaryFilter_t *f, float alpha, float T)
{
    f->alpha = alpha;
    f->dt = T;
    f->angle = 0.0f;
}

float CF_Update(ComplementaryFilter_t *f,float gyro_rate,float acc_angle)
{
    /* Integrate gyro */
    f->angle += gyro_rate * f->dt;

    /* Blend */
    f->angle = f->alpha * f->angle +
               (1.0f - f->alpha) * acc_angle;

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

    pid->out_min = out_min;
    pid->out_max = out_max;
}

/* =========================================================
 *                    PID UPDATE
 * ========================================================= */
static float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
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

    /* =============================
     * 4. Derivative
     * ============================= */
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->kd * derivative;

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
 *                LOW-PASS FILTER INITIALIZATION
 * ========================================================= */
void LPF_FirstOrder_Init(FirstOrderLPF_t *f, float cutoff_freq, float dt)
{
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
    f->cutoff_freq = cutoff_freq;
    f->alpha = dt / (RC + dt);
    f->dt = dt;
    f->d1 = 0.0f;
}

float LPF_FirstOrder_Update(FirstOrderLPF_t *f, float input)
{
    f->d1 += f->alpha * (input - f->d1);
    return f->d1;
}

void LPF_SecondOrder_Init(SecondOrderLPF_t *f, float cutoff_freq, float dt)
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

float LPF_SecondOrder_Update(SecondOrderLPF_t *f, float input)
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
    const float gyro_dt    = 1.0f/800.0f;   // 1 kHz loop
    const float accele_dt  = 1.0f/200.0f;    // 200 Hz 
    const float mag_dt     = 1.0f/25.0f;     // 25 Hz 
    const float alpha = 0.98f; 

    CF_Init(&est->roll,  alpha, gyro_dt);
    CF_Init(&est->pitch, alpha, gyro_dt);
    CF_Init(&est->yaw,   alpha, gyro_dt);

    LPF_FirstOrder_Init(&sys->accel_lpf.ax, 30.0f, accele_dt);
    LPF_FirstOrder_Init(&sys->accel_lpf.ay, 30.0f, accele_dt);
    LPF_FirstOrder_Init(&sys->accel_lpf.az, 30.0f, accele_dt);

    LPF_SecondOrder_Init(&sys->gyro_lpf.p, 80.0f, gyro_dt);
    LPF_SecondOrder_Init(&sys->gyro_lpf.q, 80.0f, gyro_dt);
    LPF_SecondOrder_Init(&sys->gyro_lpf.r, 80.0f, gyro_dt);

    LPF_FirstOrder_Init(&sys->mag_lpf.mx, 10.0f, mag_dt);
    LPF_FirstOrder_Init(&sys->mag_lpf.my, 10.0f, mag_dt);
    LPF_FirstOrder_Init(&sys->mag_lpf.mz, 10.0f, mag_dt);
    /* =====================================================
     * 2. ANGLE CONTROLLER (Outer Loop)
     * ===================================================== */
    PID_Init(&actrl->roll,  1.0f, 0.5f, 0.1f,  -30.0f, 30.0f);
    PID_Init(&actrl->pitch, 4.0f, 0.5f, 0.1f,  -30.0f, 30.0f);
    PID_Init(&actrl->yaw,   2.0f, 0.3f, 0.05f, -30.0f, 30.0f);

    /* =====================================================
     * 3. RATE CONTROLLER (Inner Loop)
     * ===================================================== */
    PID_Init(&rctrl->p, 0.1f,  0.00f,  0.000f,  -10.0f, 10.0f);
    PID_Init(&rctrl->q, 0.1f,  0.00f,  0.000f,  -10.0f, 10.0f);
    PID_Init(&rctrl->r, 0.01f, 0.000f, 0.000f, -10.0f, 10.0f);

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
    sig->motor_desired.w_fl = 0.0f;
    sig->motor_desired.w_fr = 0.0f;
    sig->motor_desired.w_bl = 0.0f;
    sig->motor_desired.w_br = 0.0f;

    sig->u_esc.w_fl = 0.0f;
    sig->u_esc.w_fr = 0.0f;
    sig->u_esc.w_bl = 0.0f;
    sig->u_esc.w_br = 0.0f;
}


void Estimator_Update(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator)
{
    /* =====================================================
     * 1. RAW SENSOR  (ALL RAD)
     * ===================================================== */

    float ax = sensor->imu.ax;
    float ay = sensor->imu.ay;
    float az = sensor->imu.az;

    float gx = sensor->imu.gx;
    float gy = sensor->imu.gy;
    float gz = sensor->imu.gz;

    float mx = sensor->imu.mx;
    float my = sensor->imu.my;
    float mz = sensor->imu.mz;

    float dt = estimator->roll.dt;
    /* =====================================================
     * 2. ROLL & PITCH 
     * ===================================================== */

    float acc_roll  = atan2f(ay, az);
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    float roll  = CF_Update(&estimator->roll,  gx, acc_roll);
    float pitch = CF_Update(&estimator->pitch, gy, acc_pitch);

    state->euler.x = roll;   
    state->euler.y = pitch;  

    /* =====================================================
     * 3. YAW 
     * ===================================================== */

    float phi   = roll;
    float theta = pitch;

    float ctheta = cosf(theta);
    if (fabsf(ctheta) < 0.01f)
        ctheta = 0.01f;

    // gyro already rad/s
    float q = gy;
    float r = gz;

    float psi_dot = (q * sinf(phi) + r * cosf(phi)) / ctheta;

    float yaw_gyro = estimator->yaw.angle + psi_dot * dt;
    yaw_gyro = wrap_pi(yaw_gyro);


    float mx_h = mx * cosf(theta) + mz * sinf(theta);

    float my_h = mx * sinf(phi) * sinf(theta)
               + my * cosf(phi)
               - mz * sinf(phi) * cosf(theta);

    float yaw_mag = -atan2f(my_h, mx_h);

    float error = wrap_pi(yaw_mag - yaw_gyro);

    float yaw = yaw_gyro +
                (1.0f - estimator->yaw.alpha) * error;

    yaw = wrap_pi(yaw);

    estimator->yaw.angle = yaw;
    state->euler.z = yaw;   // rad

    /* =====================================================
     * 4. BODY RATES (RAD/S)
     * ===================================================== */

    state->omega_body.x = gx;
    state->omega_body.y = gy;
    state->omega_body.z = gz;

    /* =====================================================
     * 5. ROTATION MATRIX (Body → World)
     * ===================================================== */

    float gamma = yaw;  // NO CONVERSION

    float cphi = cosf(phi);
    float sphi = sinf(phi);

    float stheta = sinf(theta);

    float cgamma = cosf(gamma);
    float sgamma = sinf(gamma);

    float R[9];

    R[0] = ctheta * cgamma;
    R[1] = sphi * stheta * cgamma - cphi * sgamma;
    R[2] = cphi * stheta * cgamma + sphi * sgamma;

    R[3] = ctheta * sgamma;
    R[4] = sphi * stheta * sgamma + cphi * cgamma;
    R[5] = cphi * stheta * sgamma - sphi * cgamma;

    R[6] = -stheta;
    R[7] = sphi * ctheta;
    R[8] = cphi * ctheta;

    /* =====================================================
     * 6. WORLD ACCELERATION
     * ===================================================== */

    float ax_w = R[0]*ax + R[1]*ay + R[2]*az;
    float ay_w = R[3]*ax + R[4]*ay + R[5]*az;
    float az_w = R[6]*ax + R[7]*ay + R[8]*az;

    // NED frame (+Z down)
    az_w -= GRAVITY;  // GRAVITY COMPENSATION

    state->accel_world.x = ax_w;
    state->accel_world.y = ay_w;
    state->accel_world.z = az_w;

    /* =====================================================
     * 7. WORLD ANGULAR RATES (RAD/S)
     * ===================================================== */

    float ttheta = tanf(theta);

    state->omega_world.x =
        gx + gy * sinf(phi) * ttheta
           + gz * cosf(phi) * ttheta;

    state->omega_world.y =
        gy * cosf(phi) - gz * sinf(phi);

    state->omega_world.z =
        psi_dot;   // RAD/S

    /* =====================================================
     * 8. ALTITUDE
     * ===================================================== */

    state->position.z = sensor->distance;
}

void Estimator_GyroUpdate(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator,GyroLPF_t *gyro_lpf)
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

    state->omega_body.x = gx_f;
    state->omega_body.y = gy_f;
    state->omega_body.z = gz_f;

    }

void Estimator_AccelUpdate(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, AccelLPF_t *accel_lpf)
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

    }

void Estimator_MagUpdate(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, MagLPF_t *mag_lpf)
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
}












void Motor_Mixer(Torque_t *torque_total,
                 float thrust_sp,
                 MotorCommand_t *motor_desired)
{
    float constant = sqrtf(2.0f) * 0.25f * ARM_LENGTH * KF;

    /* Base thrust → omega^2 */
    float T  = thrust_sp / (4.0f * KF);

    /* Torque → omega^2 contribution */
    float Tx = torque_total->tau_x / constant;
    float Ty = torque_total->tau_y / constant;
    float Tz = torque_total->tau_z / KM;

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
    motor_desired->w_fl = clampf(w_fl, W_MIN, W_MAX);
    motor_desired->w_fr = clampf(w_fr, W_MIN, W_MAX);
    motor_desired->w_bl = clampf(w_bl, W_MIN, W_MAX);
    motor_desired->w_br = clampf(w_br, W_MIN, W_MAX);
}

void Motor_Inverse_Model(MotorCommand_t *motor_desired,
                         MotorCommand_t *u_esc,
                         float Vbat)
{


    float u_fl = (A_COEFF * motor_desired->w_fl * motor_desired->w_fl
                 + B_COEFF * motor_desired->w_fl);

    float u_fr = (A_COEFF * motor_desired->w_fr * motor_desired->w_fr
                 + B_COEFF * motor_desired->w_fr);

    float u_bl = (A_COEFF * motor_desired->w_bl * motor_desired->w_bl
                 + B_COEFF * motor_desired->w_bl);

    float u_br = (A_COEFF * motor_desired->w_br * motor_desired->w_br
                 + B_COEFF * motor_desired->w_br);

    /* Safe division */
    u_esc->w_fl = safe_divf(u_fl, Vbat, VBAT_MIN);
    u_esc->w_fr = safe_divf(u_fr, Vbat, VBAT_MIN);
    u_esc->w_bl = safe_divf(u_bl, Vbat, VBAT_MIN);
    u_esc->w_br = safe_divf(u_br, Vbat, VBAT_MIN);

}

void ESC_Output(MotorCommand_t *u_esc) {

    // Clamp duty về [0,1]
    // Bảo vệ ESC và tránh overflow PWM
    float u_fl = clamp(u_esc->w_fl);
    float u_fr = clamp(u_esc->w_fr);
    float u_bl = clamp(u_esc->w_bl);
    float u_br = clamp(u_esc->w_br);

    // Mapping duty (0-1) sang giá trị CCR PWM
    uint16_t ccr_fl = (uint16_t)(u_fl * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_fr = (uint16_t)(u_fr * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_bl = (uint16_t)(u_bl * (CCR_MAX - CCR_MIN) + CCR_MIN);
    uint16_t ccr_br = (uint16_t)(u_br * (CCR_MAX - CCR_MIN) + CCR_MIN);

    // Xuất PWM cho 4 motor
    PWM_Motor_Write_All(ccr_fl, ccr_fr, ccr_bl, ccr_br);
}

void Controller_Rate_Loop(RateController_t *rctrl, DroneState_t *state, Torque_t *torque_pid, RateSetpoint_t *rate_sp) {
    
    torque_pid->tau_x = PID_Update(&rctrl->p, rate_sp->p , state->omega_body.x,1.0f/RATE_LOOP_HZ);
    torque_pid->tau_y = PID_Update(&rctrl->q, rate_sp->q , state->omega_body.y,1.0f/RATE_LOOP_HZ);
    torque_pid->tau_z = PID_Update(&rctrl->r, rate_sp->r , state->omega_body.z,1.0f/RATE_LOOP_HZ);


    
}