#ifndef DRIVERS_SENSOR_IMU_H_
#define DRIVERS_SENSOR_IMU_H_

#include <stdint.h>
#include <stddef.h>
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================
 * ===================== BMM150 TRIM ==========================
 * ============================================================*/
typedef struct {
    int8_t dig_x1, dig_y1, dig_x2, dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2, dig_z3, dig_z4;
    uint16_t dig_xyz1;
    int8_t dig_xy2, dig_xy1;
} BMM150_TrimData;

/* ============================================================
 * ===================== BMP388 CALIB =========================
 * ============================================================*/
typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
} BMP388_Calib;
typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

} IMU_Data_t;
typedef struct
{
    float pressure;      // Pa
    float temperature;   // °C
} Baro_Data_t;

/* ============================================================
 * ===================== BMX160 STRUCT ========================
 * ============================================================*/
typedef struct {
    BMM150_TrimData trim;
} BMX160_t;

typedef struct {
    BMP388_Calib calib;
} BMP388_t;




/*****************************************************
 *                 BMP388 DEFINITIONS
 *****************************************************/
#define BMP388_ADDRESS                  (0x76 << 1)

/* Register Map */
#define BMP388_REG_DATA                 0x04
#define BMP388_REG_IF_CONF              0x1A
#define BMP388_REG_PWR_CTRL             0x1B
#define BMP388_REG_OSR                  0x1C
#define BMP388_REG_ODR                  0x1D
#define BMP388_REG_CONFIG               0x1F
#define BMP388_REG_CALIB_DATA           0x31
#define BMP388_REG_CMD                  0x7E

/* Default Configuration Values */
#define BMP388_REG_PWR_CTRL_VALUE       0x33
#define BMP388_REG_OSR_VALUE            0x0C
#define BMP388_REG_ODR_VALUE            0x03
#define BMP388_REG_CONFIG_VALUE         0x02

/*****************************************************
 *                 BMM150 DEFINITIONS
 *****************************************************/
/* Overflow Values */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL   ((int16_t)(-4096))
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL    ((int16_t)(-16384))

/* Saturation Limits (Z-Axis, µT scale) */
#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP   INT16_C(-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL    INT16_C(-16384)
#define BMM150_OVERFLOW_OUTPUT               INT16_C(-32768)
#define BMM150_NEGATIVE_SATURATION_Z         INT16_C(-32767)
#define BMM150_POSITIVE_SATURATION_Z         INT16_C(32767)
#define BMM150_OVERFLOW_OUTPUT_FLOAT         0.0f

/*****************************************************
 *                 BMX160 DEFINITIONS
 *****************************************************/
#define BMX160_ADDRESS                  (0x68 << 1)
#define BMX160_REG_CHIP_ID 0x00
/* Data Registers */
/* ================= MAG ================= */
#define BMX160_REG_MAG_DATA_0        0x04
#define BMX160_MAG_LEN               8

/* ================= GYRO ================= */
#define BMX160_REG_GYRO_DATA_0       0x0C
#define BMX160_GYRO_LEN              6

/* ================= ACCEL ================= */
#define BMX160_REG_ACCEL_DATA_0      0x12
#define BMX160_ACCEL_LEN             6
/* Configuration Registers */
#define BMX160_REG_ACCEL_CONFIG         0x40
#define BMX160_REG_ACCEL_RANGE          0x41
#define BMX160_REG_GYRO_CONFIG          0x42
#define BMX160_REG_GYRO_RANGE           0x43
#define BMX160_REG_MAGN_CONFIG          0x44

/* Magnetometer Interface Registers */
#define BMX160_REG_MAGN_RANGE           0x4B
#define BMX160_REG_MAGN_IF_0            0x4C
#define BMX160_REG_MAGN_IF_1            0x4D
#define BMX160_REG_MAGN_IF_2            0x4E
#define BMX160_REG_MAGN_IF_3            0x4F

/* Interrupt Configuration */
#define BMX160_REG_INT_ENABLE_1         0x51
#define BMX160_REG_INT_OUT_CTRL         0x53
#define BMX160_REG_INT_MAP_1            0x56

/* Offset & FOC Configuration */
#define BMX160_REG_FOC_CONF             0x69
#define BMX160_REG_CONF                 0x6A
#define BMX160_REG_OFFSET_6             0x77

/* Command Register */
#define BMX160_REG_COMMAND              0x7E

/*****************************************************
 *                 BMX160 DEFAULT VALUES
 *****************************************************/
#define BMX160_CMD_ACCE_NORMAL_MODE     0x11
#define BMX160_CMD_GYRO_NORMAL_MODE     0x15
#define BMX160_CMD_MAG_NORMAL_MODE      0x19

#define BMX160_MAG_HIGH_ACCURACY_PRESET_XY  0x17
#define BMX160_MAG_HIGH_ACCURACY_PRESET_Z   0x52

#define BMX160_MAG_IF_0_DATA_MODE       0x03
#define BMX160_MAG_IF_1_DATA_MODE       0x42
#define BMX160_MAG_IF_2_DATA_MODE       0x4C
#define BMX160_MAG_IF_3_DATA_MODE       0x02

#define BMX160_REG_MAGN_IF_0_MANUAL      0x83
#define BMX160_ACCEL_OFFSET_SENSITIVITY  3.9e-3f   // mg/LSB
#define BMX160_GYRO_OFFSET_SENSITIVITY   0.061f    // deg/s per LSB

#define BMX160_REG_ACCEL_CONFIG_VALUE   0x19        // ODR 200Hz
#define BMX160_REG_ACCEL_RANGE_VALUE    0x03
#define BMX160_REG_GYRO_CONFIG_VALUE    0x2B        // ODR 1000Hz
#define BMX160_REG_GYRO_RANGE_VALUE     0x04        
#define BMX160_REG_MAGN_CONFIG_VALUE    0x06
#define BMX160_REG_INT_ENABLE_1_VALUE   0x10
#define BMX160_REG_INT_OUT_CTRL_VALUE   0x08
#define BMX160_REG_INT_MAP_1_VALUE      0x80
#define BMX160_REG_FOC_CONF_VALUE       0x7D
#define BMX160_REG_OFFSET_6_VALUE       0xC0
#define BMX160_REG_MAGN_IF_0_VALUE      0x80
/* ============================================================
 * ===================== BMX160 DEFINITIONS ===================
 * ============================================================*/

#define BMX160_ADDRESS (0x68 << 1)
#define BMX160_LEN 21
#define BMX160_OFFSET 0

#define BMX160_CMD_ACCE_NORMAL_MODE 0x11
#define BMX160_CMD_GYRO_NORMAL_MODE 0x15
#define BMX160_CMD_MAG_NORMAL_MODE 0x19
#define BMX160_CMD_START_FOC 0x03
#define BMX160_REG_PMU_STATUS 0x03
#define BMX160_CMD_FLUSH_FIFO 0xB0
/* ============================================================
 * ===================== BMP388 DEFINITIONS ===================
 * ============================================================*/

#define BMP388_ADDRESS (0x76 << 1)
#define BMP388_LEN 6
#define BMP388_OFFSET (BMX160_OFFSET + BMX160_LEN)
#define BMX160_REG_ERR 0x02
/* ============================================================
 * ===================== MAG CALIB ============================
 * ============================================================*/

#define MAG_OX -7.27317214f
#define MAG_OY 0.72604599f
#define MAG_OZ 35.24629018f

#define MAG_M11 1.0f
#define MAG_M12 0.0f
#define MAG_M13 0.0f

#define MAG_M21 0.0f
#define MAG_M22 1.0f
#define MAG_M23 0.0f

#define MAG_M31 0.0f
#define MAG_M32 0.0f
#define MAG_M33 1.0f

/* ============================================================
 * ===================== FUNCTION PROTOTYPES ==================
 * ============================================================*/
#define phi_alpha    0.95f
#define theta_alpha  0.95f
#define gamma_alpha  0.96f

void bmx160_init(void);
void Read_Gyro(IMU_Data_t *imu_data);
void Read_Accel(IMU_Data_t *imu_data);
void Read_Magnetic(IMU_Data_t *imu_data);
void bmp388_init(void);
void Read_Barometer(Baro_Data_t *baro);
void IMU_Init(void);
#endif /* DRIVERS_SENSOR_IMU_H_ */
