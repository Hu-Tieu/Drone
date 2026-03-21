#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <math.h>
#include "IMU/imu.h"
#include "Receiver/receiver.h"
/* =========================================================
 * Quadcopter Physical Parameters
 * ========================================================= */

/* ================= Geometry ================= */
#define ARM_LENGTH              0.215f      // Distance from CoG to each motor (m)

/* ================= Mass & Gravity ================= */
#define DRONE_MASS              1.245f      // kg
#define GRAVITY                 9.807f      // m/s^2

/* ================= Inertia Matrix (kg·m^2) ================= */
#define Jxx                     0.01438644f     // Moment of inertia around X axis
#define Jyy                     0.01518077f     // Moment of inertia around Y axis
#define Jzz                     0.02827181f     // Moment of inertia around Z axis

/* ================= Aerodynamic Coefficients ================= */
/* Thrust model: F = KF * omega^2 */
#define KF                      1.05e-5f    // N / (rad/s)^2

/* Drag torque model: Tau_z = KM * omega^2 */
#define KM                      1.0e-7f     // N·m / (rad/s)^2

/* ================= Motor Characteristics ================= */
#define MOTOR_COUNT             4
#define MOTOR_THRUST_MAX_N      10.6f       // 1 motor at PWM100 (N)
#define THRUST_HEADROOM         0.8f        // Use 80% max thrust for control authority

/* ================= Motor Speed Limits ================= */
#define W_MIN                   0.0f        // rad/s
#define W_MAX                   900.0f      // rad/s (adjust to your motor)

/* ================= PWM / Duty Limits ================= */
#define DUTY_MIN                0.0f
#define DUTY_MAX                1.0f

/* ================= Battery Protection ================= */
#define VBAT_MIN                10.5f        // Avoid division by zero

/* ================= Motor Inverse Model ================= */
#define A_COEFF                 4.715248338627949e-06
#define B_COEFF                 0.007846454313235452
#define C_COEFF                 0.001f
/* ============================================================
 *                     SENSOR STRUCTURE
 *   (Main ghi trực tiếp dữ liệu cảm biến vào đây)
 * ============================================================ */
#define GYRO_ANGLE 1
#define ACCEL_ANGLE 2
#define MAG_ANGLE 3
/* ================= PID Parameters ================= */
#define ROLL_KP                 0.0f
#define ROLL_KI                 0.0f
#define ROLL_KD                 0.0f

#define PITCH_KP                0.0f
#define PITCH_KI                0.0f
#define PITCH_KD                0.0f

#define YAW_KP                  0.0f
#define YAW_KI                  0.0f
#define YAW_KD                  0.0f

#define P_KP                    0.0f
#define P_KI                    0.0f
#define P_KD                    0.0f

#define Q_KP                    0.0f
#define Q_KI                    0.0f
#define Q_KD                    0.0f

#define R_KP                    0.0f
#define R_KI                    0.0f
#define R_KD                    0.0f
/* ================= PID ================= */
typedef struct {

    IMU_Data_t imu;
    Baro_Data_t baro;
    IBUS_ChannelData ibus;

    float battery;
    float motor_rpm;
    long  distance;

} DroneSensor_t;

/* ============================================================
 *                      BASIC VECTOR
 * ============================================================ */

typedef struct {
    float x;
    float y;
    float z;
} Vec3f_t;

/* ============================================================
 *                  ATTITUDE ESTIMATOR
 * ============================================================ */

typedef struct {
    float alpha;
    float dt;
    float angle;
} ComplementaryFilter_t;

typedef struct {
    ComplementaryFilter_t phi;
    ComplementaryFilter_t theta;
    ComplementaryFilter_t psi;

} AttitudeEstimator_t;
/* ============================================================
 *                  LOW-PASS FILTER
 * ============================================================ */

 typedef struct {
    float cutoff_freq;
    float dt;
    float alpha;
    float d1;
} FirstOrderLPF_t;

typedef struct {
    float cutoff_freq;
    float dt;
    float b0, b1,b2;
    float a1, a2;
    float d1, d2;
} SecondOrderLPF_t;

typedef struct {
    FirstOrderLPF_t ax;
    FirstOrderLPF_t ay;
    FirstOrderLPF_t az;
} AccelLPF_t;

typedef struct {
    SecondOrderLPF_t p;
    SecondOrderLPF_t q;
    SecondOrderLPF_t r;
} GyroLPF_t;

typedef struct {
    FirstOrderLPF_t mx;
    FirstOrderLPF_t my;
    FirstOrderLPF_t mz;
} MagLPF_t;

/* ============================================================
 *                      PID
 * ============================================================ */

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;

    float out_min;
    float out_max;
} PID_t;

/* ============================================================
 *                      COMMANDS
 * ============================================================ */

typedef struct {
    float roll;
    float pitch;
    float yaw;
} AngleSetpoint_t;

typedef struct {
    float p;
    float q;
    float r;
} RateSetpoint_t;

typedef struct {
    float vx;
    float vy;
    float vz;
} VelocitySetpoint_t;

typedef struct {
    float x;
    float y;
    float z;
} PositionSetpoint_t;

typedef struct {
    float tau_x;
    float tau_y;
    float tau_z;
} Torque_t;

typedef struct {
    float w_fl;
    float w_fr;
    float w_bl;
    float w_br;
} MotorCommand_t;

/* ============================================================
 *                      CONTROLLERS
 * ============================================================ */

typedef struct {
    PID_t roll;
    PID_t pitch;
    PID_t yaw;
} AngleController_t;

typedef struct {
    PID_t p;
    PID_t q;
    PID_t r;
} RateController_t;

typedef struct {
    PID_t vx;
    PID_t vy;
    PID_t vz;
} VelocityController_t;

typedef struct {
    PID_t x;
    PID_t y;
    PID_t z;
} PositionController_t;

typedef struct {
    float tau_x;
    float tau_y;
    float tau_z;
} FeedForward_t;

/* ============================================================
 *                      INERTIA
 * ============================================================ */

typedef struct {
    float Jx;
    float Jy;
    float Jz;
} Inertia_t;

/* ============================================================
 *                  CONTROL SIGNAL PIPELINE
 * ============================================================ */

typedef struct {

    AngleSetpoint_t angle_sp;
    RateSetpoint_t  rate_sp;
    VelocitySetpoint_t velocity_sp;
    PositionSetpoint_t position_sp;

    Torque_t torque_pid;
    Torque_t torque_ff;
    Torque_t torque_total;

    float thrust_sp;

    MotorCommand_t motor_desired;
    MotorCommand_t u_esc;

} ControlSignals_t;
/* ============================================================
 *                      DRONE STATE
 *      Full 6DOF physical state of the vehicle
 * ============================================================ */

typedef struct
{
    /* =========================
     * ORIENTATION
     * ========================= */

    Vec3f_t euler;          // [deg]  roll (φ), pitch (θ), yaw (ψ)

    /* =========================
     * ANGULAR MOTION
     * ========================= */

    Vec3f_t omega_body;     // [deg/s] body rates (p, q, r)
    Vec3f_t omega_world;    // [deg/s] euler angle rates (φ̇, θ̇, ψ̇)

    /* =========================
     * TRANSLATIONAL MOTION
     * ========================= */

    Vec3f_t position;       // [m]    world position (x, y, z)
    Vec3f_t velocity;       // [m/s]  world velocity

    /* =========================
     * ACCELERATION
     * ========================= */

    Vec3f_t accel_body;     // [m/s²] raw accel (IMU frame)
    Vec3f_t accel_world;    // [m/s²] gravity-compensated world accel
    /* Estimated gravity magnitude used for compensation (m/s^2) */
    float gravity_est;

    /* =========================
     * MAGNETOMETER
     * ========================= */
    Vec3f_t mag;            // [uT]   magnetometer readings
    
    /* =========================
     * HEIGHT
     * ========================= */
    float baro_altitude;    // [m]    barometric altitude
    float lazer_altitude;   // [m]    laser rangefinder altitude

    /* =========================
     * CONTROL-RELATED
     * ========================= */

    float total_thrust;     // [N] total thrust produced by motors

} DroneState_t;
/* ============================================================
 *                      MAIN CONTROL SYSTEM
 * ============================================================ */

typedef struct {

    /* === Raw sensor input === */
    DroneSensor_t sensor;
    DroneState_t state;
    /*=== Low-pass Filters === */
    AccelLPF_t accel_lpf;
    GyroLPF_t gyro_lpf;
    MagLPF_t mag_lpf;
    FirstOrderLPF_t baro_lpf;
    /* === Estimation result === */
    AttitudeEstimator_t estimator;
    Vec3f_t attitude;     // roll pitch yaw (deg)
    Vec3f_t rates;        // p q r (deg/s)

    /* === Controllers === */
    AngleController_t angle_ctrl;
    RateController_t  rate_ctrl;
    VelocityController_t velocity_ctrl;
    PositionController_t position_ctrl;

    Inertia_t inertia;
    FeedForward_t feedforward;

    /* === Control signals === */
    ControlSignals_t signals;

} ControlSystem_t;

/* ============================================================
 *                      API
 * ============================================================ */

/* --- Complementary Filter --- */
void Controller_Init(ControlSystem_t *sys);
void Gyro_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator,GyroLPF_t *gyro_lpf);
void Accel_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, AccelLPF_t *accel_lpf);
void Mag_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, MagLPF_t *mag_lpf);
void Barometer_LPF(DroneSensor_t *sensor,DroneState_t *state, AttitudeEstimator_t *estimator, FirstOrderLPF_t *baro_lpf);
/* --- Sensor --- */
void Receiver_Mapping(DroneSensor_t *sensor, ControlSignals_t *signals);
void ESC_Force(MotorCommand_t *u_esc);
/* --- Estimator --- */
void Estimator_Angle(DroneState_t *state, AttitudeEstimator_t *estimator,uint8_t TYPE);
void Convert_BodyToWorld(DroneState_t *state);
void ESC_Output(MotorCommand_t *u_esc);
void Motor_Inverse_Model(MotorCommand_t *motor_desired,MotorCommand_t *u_esc,float Vbat);
void Motor_Mixer(Torque_t *torque_total, float thrust_sp, MotorCommand_t *motor_desired);
void Rate_PID_Reset(RateController_t *rctrl);
void Angle_PID_Reset(AngleController_t *actrl);
/* --- Control Loops --- */
void Controller_Rate_Loop(RateController_t *rctrl, DroneState_t *state, Torque_t *torque_pid, RateSetpoint_t *rate_sp);
void Controller_Angle_Loop(ControlSystem_t *sys);
void Controller_Mixer(ControlSystem_t *sys);

#endif