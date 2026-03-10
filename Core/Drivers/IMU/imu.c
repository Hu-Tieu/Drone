#include "imu.h"
#include "i2c.h"
#include "main.h"
#include <string.h>

I2C_HandleTypeDef *hi2c3_IMU      = &hi2c3;

static BMX160_t g_bmx160;
static BMP388_t g_bmp388;

static void I2C_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Enable clock GPIO nếu chưa
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 2. Config SCL (PA8) và SDA (PC9) thành Open-Drain Output
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // SCL
    GPIO_InitStruct.Pin = IMU_CLK_Pin;
    HAL_GPIO_Init(IMU_CLK_GPIO_Port, &GPIO_InitStruct);

    // SDA
    GPIO_InitStruct.Pin = IMU_SDA_Pin;
    HAL_GPIO_Init(IMU_SDA_GPIO_Port, &GPIO_InitStruct);

    // 3. Đảm bảo SDA đang HIGH
    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // 4. Toggle SCL 9 lần để giải phóng slave
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(IMU_CLK_GPIO_Port, IMU_CLK_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);

        HAL_GPIO_WritePin(IMU_CLK_GPIO_Port, IMU_CLK_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 5. Tạo STOP condition thủ công
    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_CLK_GPIO_Port, IMU_CLK_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static HAL_StatusTypeDef bmx160_write(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c3_IMU,
                             BMX160_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1,
                             HAL_MAX_DELAY);
}

static HAL_StatusTypeDef bmx160_read(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(hi2c3_IMU,
                            BMX160_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            1,
                            HAL_MAX_DELAY);
}

static uint8_t bmx160_check_id(void)
{
    uint8_t id = 0;
    bmx160_read(BMX160_REG_CHIP_ID, &id);
    return id;   // phải = 0xD8
}

static uint8_t bmx160_read_pmu(void)
{
    uint8_t status = 0;
    bmx160_read(BMX160_REG_PMU_STATUS, &status);
    return status;
}

static void bmx160_soft_reset(void)
{
    bmx160_write(BMX160_REG_COMMAND, 0xB6);
    HAL_Delay(20);   // >= 10ms
}

static void bmx160_set_accel_normal(void)
{
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_ACCE_NORMAL_MODE);
    HAL_Delay(5);
}

static void bmx160_set_gyro_normal(void)
{
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_GYRO_NORMAL_MODE);
    HAL_Delay(80);
}

static void bmx160_set_mag_normal(void)
{
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_MAG_NORMAL_MODE);
    HAL_Delay(50);
}

static void bmm150_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_I2C_Mem_Write(hi2c3_IMU, BMX160_ADDRESS,
    BMX160_REG_MAGN_IF_1,
    I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY);

    while (1)
    {
        uint8_t status = bmx160_read_pmu();
        if ((status & 0x04) == 0x04) { // bit 1: MAG_RDY
            break;
        }
    }
    HAL_Delay(1);
    HAL_I2C_Mem_Read(hi2c3_IMU, BMX160_ADDRESS,
    BMX160_REG_MAG_DATA_0,
    I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

static void bmm150_write(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c3_IMU, BMX160_ADDRESS, BMX160_REG_MAGN_IF_3,
            I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);    

    HAL_I2C_Mem_Write(hi2c3_IMU, BMX160_ADDRESS, BMX160_REG_MAGN_IF_2,
            I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY);

    while (1)
    {
        uint8_t status = bmx160_read_pmu();
        if ((status & 0x04) == 0x04) { // bit 1: MAG_WRDY
            break;
        }
    }
}

static void bmm150_read_trim(BMX160_t *t) {
    uint8_t buf[2];

    // dig_x1, dig_y1
    bmm150_read(0x5D, buf, 2);
    t->trim.dig_x1 = (int8_t) buf[0];
    t->trim.dig_y1 = (int8_t) buf[1];
    // dig_z4
    bmm150_read(0x62, buf, 2);
    t->trim.dig_z4 = (int16_t) ((buf[1] << 8) | buf[0]);

    // dig_x2, dig_y2
    bmm150_read(0x64, buf, 2);
    t->trim.dig_x2 = (int8_t) buf[0];
    t->trim.dig_y2 = (int8_t) buf[1];

    // dig_z2
    bmm150_read(0x68, buf, 2);
    t->trim.dig_z2 = (int16_t) ((buf[1] << 8) | buf[0]);

    // dig_z1
    bmm150_read(0x6A, buf, 2);
    t->trim.dig_z1 = (uint16_t) ((buf[1] << 8) | buf[0]);

    // dig_xyz1
    bmm150_read(0x6C, buf, 2);
    t->trim.dig_xyz1 = (uint16_t) ((buf[1] << 8) | buf[0]);

    // dig_z3
    bmm150_read(0x6E, buf, 2);
    t->trim.dig_z3 = (int16_t) ((buf[1] << 8) | buf[0]);

    // dig_xy2, dig_xy1
    bmm150_read(0x70, buf, 2);
    t->trim.dig_xy2 = (int8_t) buf[0];
    t->trim.dig_xy1 = (int8_t) buf[1];
}

void bmp388_init(void) {
    uint8_t cmd;

    // 1. Set power control
    cmd = BMP388_REG_PWR_CTRL_VALUE;
    HAL_I2C_Mem_Write(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_PWR_CTRL,
            I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(2);

    // 2. Set oversampling
    cmd = BMP388_REG_OSR_VALUE;
    HAL_I2C_Mem_Write(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_OSR,
            I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(2);

    // 3. Set ODR
    cmd = BMP388_REG_ODR_VALUE;
    HAL_I2C_Mem_Write(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_ODR,
            I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(2);

    // 4. Set config
    cmd = BMP388_REG_CONFIG_VALUE;
    HAL_I2C_Mem_Write(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_CONFIG,
            I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(2);
    uint8_t data[30];
    // 5. Read calibration data (21 bytes)
    HAL_I2C_Mem_Read(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_CALIB_DATA,
            I2C_MEMADD_SIZE_8BIT, &data[0], 21, HAL_MAX_DELAY);

    uint16_t NVM_PAR_T1 = ((uint16_t) data[1] << 8) | data[0];
    uint16_t NVM_PAR_T2 = ((uint16_t) data[3] << 8) | data[2];
    int8_t NVM_PAR_T3 = data[4];

    int16_t NVM_PAR_P1 = ((int16_t) data[6] << 8) | data[5];
    int16_t NVM_PAR_P2 = ((int16_t) data[8] << 8) | data[7];
    int8_t NVM_PAR_P3 = data[9];
    int8_t NVM_PAR_P4 = data[10];
    uint16_t NVM_PAR_P5 = ((uint16_t) data[12] << 8) | data[11];
    uint16_t NVM_PAR_P6 = ((uint16_t) data[14] << 8) | data[13];
    int8_t NVM_PAR_P7 = data[15];
    int8_t NVM_PAR_P8 = data[16];
    int16_t NVM_PAR_P9 = ((int16_t) data[18] << 8) | data[17];
    int8_t NVM_PAR_P10 = data[19];
    int8_t NVM_PAR_P11 = data[20];

    // 6. Compute actual calibration parameters (the formulas từ datasheet)
    g_bmp388.calib.par_t1 = NVM_PAR_T1 * 256.0f;          // 2^-8
    g_bmp388.calib.par_t2 = NVM_PAR_T2 / (float) (1UL << 30);
    g_bmp388.calib.par_t3 = NVM_PAR_T3 / (float) (1ULL << 48);

    g_bmp388.calib.par_p1 = (NVM_PAR_P1 - (1 << 14)) / (float) (1 << 20);
    g_bmp388.calib.par_p2 = (NVM_PAR_P2 - (1 << 14)) / (float) (1 << 29);
    g_bmp388.calib.par_p3 = NVM_PAR_P3 / (float) (1ULL << 32);        // 2^32
    g_bmp388.calib.par_p4 = NVM_PAR_P4 / (float) (1ULL << 37);
    g_bmp388.calib.par_p5 = NVM_PAR_P5 * 8.0f;            // 2^-3
    g_bmp388.calib.par_p6 = NVM_PAR_P6 / 64.0f;           // 2^6
    g_bmp388.calib.par_p7 = NVM_PAR_P7 / 256.0f;          // 2^8
    g_bmp388.calib.par_p8 = NVM_PAR_P8 / 32768.0f;        // 2^15
    g_bmp388.calib.par_p9 = NVM_PAR_P9 / (float) (1ULL << 48);
    g_bmp388.calib.par_p10 = NVM_PAR_P10 / (float) (1ULL << 48);
    g_bmp388.calib.par_p11 = NVM_PAR_P11 / 36893488147419103232.0f; // 2^65
}

void Read_Barometer(Baro_Data_t *baro) {
    uint8_t data[BMP388_LEN];
    HAL_I2C_Mem_Read(hi2c3_IMU, BMP388_ADDRESS, BMP388_REG_DATA,
            I2C_MEMADD_SIZE_8BIT, data, BMP388_LEN,
            HAL_MAX_DELAY);
    uint32_t p_raw = (data[0]) | (data[1] << 8) | (data[2] << 16);
    int32_t t_raw = (int32_t)(((uint32_t)data[3]) | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16));

    // 2. Chuyển sang float
    float t_adc = (float) t_raw;
    float p_adc = (float) p_raw;

    // 3. Bù trừ nhiệt độ theo datasheet
    float partial_data1 = (t_adc - (float) g_bmp388.calib.par_t1);
    float partial_data2 = partial_data1 * g_bmp388.calib.par_t2;
    float temperature = partial_data2
            + (partial_data1 * partial_data1) * g_bmp388.calib.par_t3;

    // 4. Bù trừ áp suất theo datasheet
    float partial_data3 = g_bmp388.calib.par_p6 * temperature;
    float partial_data4 = g_bmp388.calib.par_p7 * temperature * temperature;
    float partial_data5 = g_bmp388.calib.par_p8 * temperature * temperature
            * temperature;
    float partial_out1 = g_bmp388.calib.par_p5 + partial_data3 + partial_data4
            + partial_data5;

    float partial_data6 = g_bmp388.calib.par_p2 * temperature;
    float partial_data7 = g_bmp388.calib.par_p3 * temperature * temperature;
    float partial_data8 = g_bmp388.calib.par_p4 * temperature * temperature
            * temperature;
    float partial_out2 =
            p_adc
                    * (g_bmp388.calib.par_p1 + partial_data6 + partial_data7
                            + partial_data8);

    float partial_out3 = p_adc * p_adc
            * (g_bmp388.calib.par_p9 + g_bmp388.calib.par_p10 * temperature);
    float pressure = partial_out1 + partial_out2 + partial_out3
            + g_bmp388.calib.par_p11 * p_adc * p_adc * p_adc;

    // 5. Lưu kết quả
    baro->temperature = temperature;  // °C
    baro->pressure= pressure;     // Pa
}

void bmx160_init(void) {
        /* 1. Reset */
    HAL_I2C_DeInit(hi2c3_IMU);
    I2C_BusRecovery();
    HAL_I2C_Init(hi2c3_IMU);
    HAL_Delay(10);
	memset(&g_bmx160.trim, 0, sizeof(g_bmx160.trim));
    bmx160_soft_reset();
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_FLUSH_FIFO);
    /* 2. Check ID */
    if (bmx160_check_id() != 0xD8)
    {
        // handle error
        return;
    }
    bmx160_write(BMX160_REG_COMMAND, 0x10);
    HAL_Delay(2);
    bmx160_write(BMX160_REG_COMMAND, 0x14);
    HAL_Delay(2);
    bmx160_write(BMX160_REG_COMMAND, 0x18);
    HAL_Delay(2);

    bmx160_set_accel_normal();
    bmx160_set_gyro_normal();
    bmx160_set_mag_normal();

    /* 4. Config accel */
    bmx160_write(BMX160_REG_ACCEL_CONFIG, BMX160_REG_ACCEL_CONFIG_VALUE);
    bmx160_write(BMX160_REG_ACCEL_RANGE,  BMX160_REG_ACCEL_RANGE_VALUE);

    /* 5. Config gyro */
    bmx160_write(BMX160_REG_GYRO_CONFIG,  BMX160_REG_GYRO_CONFIG_VALUE);
    bmx160_write(BMX160_REG_GYRO_RANGE,   BMX160_REG_GYRO_RANGE_VALUE);

    bmx160_write(BMX160_REG_FOC_CONF, BMX160_REG_FOC_CONF_VALUE);
    bmx160_write(BMX160_REG_OFFSET_6, BMX160_REG_OFFSET_6_VALUE);
    /* 6. Config mag */
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_MAG_NORMAL_MODE);
    HAL_Delay(1);
    bmx160_write(BMX160_REG_MAGN_IF_0, 0x83);
    HAL_Delay(1);

    bmm150_read_trim(&g_bmx160);
    bmm150_write(0x4B, 0x01);   // Power on
    bmm150_write(0x4C, 0x00);   // Normal mode
    bmm150_write(0x51, 0x04); // enable mag data readout
    bmm150_write(0x52, 0x0E); // enable mag data readout

    bmx160_write(BMX160_REG_MAGN_IF_3, BMX160_MAG_IF_3_DATA_MODE);
    bmx160_write(BMX160_REG_MAGN_IF_2, BMX160_MAG_IF_2_DATA_MODE);
    bmx160_write(BMX160_REG_MAGN_IF_1, BMX160_MAG_IF_1_DATA_MODE);
    bmx160_write(BMX160_REG_MAGN_CONFIG, BMX160_REG_MAGN_CONFIG_VALUE);
    bmx160_write(BMX160_REG_MAGN_IF_0, BMX160_MAG_IF_0_DATA_MODE);
    bmx160_write(BMX160_REG_COMMAND, BMX160_CMD_START_FOC);
    HAL_Delay(2);

}

void Read_Gyro_Accele(IMU_Data_t *imu)
{
    uint8_t data[BMX160_ACCEL_LEN + BMX160_GYRO_LEN];

    HAL_I2C_Mem_Read(hi2c3_IMU,
                     BMX160_ADDRESS,
                     BMX160_REG_GYRO_DATA_0,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     BMX160_ACCEL_LEN + BMX160_GYRO_LEN,
                     HAL_MAX_DELAY);

    /* ===== Gyro ===== */
    int16_t gx = (int16_t)(data[0] | (data[1] << 8));
    int16_t gy = (int16_t)(data[2] | (data[3] << 8));
    int16_t gz = (int16_t)(data[4] | (data[5] << 8));

    imu->gy =  gx * 3.8e-3f * M_PI / 180.0f; // dps → rad/s
    imu->gx =  gy * 3.8e-3f * M_PI / 180.0f;
    imu->gz = -gz * 3.8e-3f * M_PI / 180.0f;

    /* ===== Accel ===== */
    int16_t ax = (int16_t)(data[6]  | (data[7]  << 8));
    int16_t ay = (int16_t)(data[8]  | (data[9]  << 8));
    int16_t az = (int16_t)(data[10] | (data[11] << 8));

    imu->ay = -ax * 6.1e-5f * 9.807f; // LSB → m/s²
    imu->ax = -ay * 6.1e-5f * 9.807f;
    imu->az =  az * 6.1e-5f * 9.807f;
    
}

void Read_Magnetic(IMU_Data_t *imu_data)
{
    uint8_t data[BMX160_MAG_LEN];

    HAL_I2C_Mem_Read(hi2c3_IMU,BMX160_ADDRESS,BMX160_REG_MAG_DATA_0,
                     I2C_MEMADD_SIZE_8BIT,data,BMX160_MAG_LEN,HAL_MAX_DELAY);
    
    int16_t raw_x, raw_y, raw_z;
    uint16_t raw_rhall;

    /* X axis */
    uint8_t lsb_x = (data[0] & 0xF8) >> 3;   // chỉ lấy bit 7:3
    int8_t msb_x = (int8_t) data[1];          // full 8 bit có dấu
    raw_x = ((int16_t) (msb_x * 32)) | lsb_x; // <<5

    /* Y axis */
    uint8_t lsb_y = (data[2] & 0xF8) >> 3;
    int8_t msb_y = (int8_t) data[3];
    raw_y = ((int16_t) (msb_y * 32)) | lsb_y;

    /* Z axis */
    uint8_t lsb_z = (data[4] & 0xFE) >> 1;
    int8_t msb_z = (int8_t) data[5];
    raw_z = ((int16_t) (msb_z <<7)) | lsb_z;
    /* RHALL */
    uint8_t lsb_r = (data[6] & 0xFC) >> 2;
    uint8_t msb_r = data[7];
    raw_rhall = ((uint16_t) msb_r << 6) | lsb_r;

    float comp_x = 0.0f;
    float comp_y = 0.0f;
    float comp_z = 0.0f;

    /* ===================== X ===================== */
    if (raw_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
    {
        float r = (raw_rhall != 0) ?
                  (float)raw_rhall :
                  (float)g_bmx160.trim.dig_xyz1;

        if (r != 0.0f)
        {
            float tmp = ((float)g_bmx160.trim.dig_xyz1 * 16384.0f / r) - 16384.0f;

            float val =
                (((float)g_bmx160.trim.dig_xy2 * tmp * tmp / 128.0f) +
                 (tmp * (float)g_bmx160.trim.dig_xy1 * 128.0f)) / 512.0f
                + 1048576.0f;

            val = val * ((float)g_bmx160.trim.dig_x2 + 160.0f) / 4096.0f;

            comp_x = ((float)raw_x * val / 8192.0f
                      + (float)g_bmx160.trim.dig_x1 * 8.0f) / 16.0f;
        }
    }

    /* ===================== Y ===================== */
    if (raw_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
    {
        float r = (raw_rhall != 0) ?
                  (float)raw_rhall :
                  (float)g_bmx160.trim.dig_xyz1;

        if (r != 0.0f)
        {
            float tmp = ((float)g_bmx160.trim.dig_xyz1 * 16384.0f / r) - 16384.0f;

            float val =
                (((float)g_bmx160.trim.dig_xy2 * tmp * tmp / 128.0f) +
                 (tmp * (float)g_bmx160.trim.dig_xy1 * 128.0f)) / 512.0f
                + 1048576.0f;

            val = val * ((float)g_bmx160.trim.dig_y2 + 160.0f) / 4096.0f;

            comp_y = ((float)raw_y * val / 8192.0f
                      + (float)g_bmx160.trim.dig_y1 * 8.0f) / 16.0f;
        }
    }

    /* ===================== Z ===================== */
    if (raw_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL)
    {
        if (g_bmx160.trim.dig_z1 != 0 &&
            g_bmx160.trim.dig_z2 != 0 &&
            g_bmx160.trim.dig_xyz1 != 0 &&
            raw_rhall != 0)
        {
            float tmp = (float)raw_rhall - (float)g_bmx160.trim.dig_xyz1;

            float val = ((float)g_bmx160.trim.dig_z3 * tmp) / 4.0f;

            float z = ((float)raw_z - (float)g_bmx160.trim.dig_z4) * 32768.0f;

            float tmp2 =
                ((float)g_bmx160.trim.dig_z1 *
                 (float)raw_rhall * 2.0f + 32768.0f) / 65536.0f;

            comp_z = (z - val) /
                     ((float)g_bmx160.trim.dig_z2 + tmp2) / 16.0f;

            if (comp_z > BMM150_POSITIVE_SATURATION_Z)
                comp_z = BMM150_POSITIVE_SATURATION_Z;

            if (comp_z < BMM150_NEGATIVE_SATURATION_Z)
                comp_z = BMM150_NEGATIVE_SATURATION_Z;
        }
    }

    /* ===================== Axis swap ===================== */
    float mx = comp_x + MAG_OX;
    float my = comp_y + MAG_OY;
    float mz = comp_z + MAG_OZ;

    /* SOFT IRON (matrix 3x3) */
    float mx_cal = MAG_M11*mx + MAG_M12*my + MAG_M13*mz;
    float my_cal = MAG_M21*mx + MAG_M22*my + MAG_M23*mz;
    float mz_cal = MAG_M31*mx + MAG_M32*my + MAG_M33*mz;

    /* lưu lại */
    imu_data->mx = my_cal;
    imu_data->my = mx_cal;
    imu_data->mz = -mz_cal;

}

void IMU_Init(void) {
    bmx160_init();
    bmp388_init();
}