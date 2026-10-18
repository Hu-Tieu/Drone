#ifndef DRIVERS_RF24_H_
#define DRIVERS_RF24_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "controller_main.h"


#define PID_RX_SCALE 100.0f

/* =========================================================
 * NRF24L01 COMMANDS
 * ========================================================= */

#define CMD_R_REGISTER      0x00
#define CMD_W_REGISTER      0x20
#define CMD_R_RX_PAYLOAD    0x61
#define CMD_W_TX_PAYLOAD    0xA0
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2
#define CMD_REUSE_TX_PL     0xE3
#define CMD_R_RX_PL_WID     0x60
#define CMD_ACTIVATE        0x50
#define CMD_NOP             0xFF

/* =========================================================
 * REGISTER MAP
 * ========================================================= */

#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_AW        0x03
#define REG_SETUP_RETR      0x04
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_OBSERVE_TX      0x08
#define REG_RX_ADDR_P0      0x0A
#define REG_TX_ADDR         0x10
#define REG_RX_PW_P0        0x11
#define REG_FIFO_STATUS     0x17
#define REG_DYNPD           0x1C
#define REG_FEATURE         0x1D

/* =========================================================
 * FIFO STATUS BITS
 * ========================================================= */

#define FIFO_TX_REUSE       (1 << 6)
#define FIFO_TX_FULL        (1 << 5)
#define FIFO_TX_EMPTY       (1 << 4)
#define FIFO_RX_FULL        (1 << 1)
#define FIFO_RX_EMPTY       (1 << 0)

#define FIFO_STATUS_TX_EMPTY 0x10

/* =========================================================
 * STATUS FLAGS
 * ========================================================= */

#define STATUS_RX_DR        0x40
#define STATUS_TX_DS        0x20
#define STATUS_MAX_RT       0x10
#define STATUS_TX_FULL      0x01

#define STATUS_CLEAR_ALL    0x70

/* =========================================================
 * RF24 TELEMETRY CONFIG
 * ========================================================= */

#define RF24_CHUNK_SIZE     32
#define RF24_CHUNK_COUNT    3
#define RF24_FRAME_SIZE     96

/* =========================================================
 * GPIO PINS 
 * ========================================================= */

#define RF24_CE_LOW()   HAL_GPIO_WritePin(RF24_CE_GPIO_Port, RF24_CE_Pin, GPIO_PIN_RESET)
#define RF24_CE_HIGH()  HAL_GPIO_WritePin(RF24_CE_GPIO_Port, RF24_CE_Pin, GPIO_PIN_SET)

#define RF24_CSN_LOW()  HAL_GPIO_WritePin(RF24_CSN_GPIO_Port, RF24_CSN_Pin, GPIO_PIN_RESET)
#define RF24_CSN_HIGH() HAL_GPIO_WritePin(RF24_CSN_GPIO_Port, RF24_CSN_Pin, GPIO_PIN_SET)
/* =========================================================
 * PACKET ID
 * ========================================================= */

typedef enum
{
    PKT_FRAME96 = 0x01

} PacketID_t;

typedef enum
{
    RX_PID_RATE_ANGLE_ID = 0x1001,
    RX_PID_POS_VEL_ID    = 0x1002,
    RX_SETTING_CMD_ID    = 0x2001

} RxPidPacketID_t;

typedef enum
{
    SETTING_CMD_RESET_ANGLE = 0x0001,
    SETTING_CMD_RESET_RATE  = 0x0002,
    SETTING_CMD_RESET_SENSORS = 0x0003
} SettingCommand_t;

typedef enum
{
    ACK_STATUS_OK       = 0x01,
    ACK_STATUS_BAD_ID   = 0xE1,
    ACK_STATUS_BAD_SIZE = 0xE2

} AckStatus_t;

typedef struct __attribute__((packed))
{
    uint8_t  id;
    uint16_t sequence;
    uint8_t  chunk_idx;
    uint8_t  status;

} AckPayload_t;

typedef struct __attribute__((packed))
{
    uint16_t id;
    uint32_t sequence;

    int16_t rate[3][3];
    int16_t angle[3][3];

} RxPidRateAngle_t;

typedef struct __attribute__((packed))
{
    uint16_t id;
    uint32_t sequence;

    int16_t position[3][3];
    int16_t velocity[3][3];

} RxPidPosVel_t;

typedef struct __attribute__((packed))
{
    uint16_t id;
    uint32_t sequence;
    uint16_t command;
    int16_t value;
} RxSettingCmd_t;

typedef struct
{
    uint8_t active;
    uint16_t id;
    uint16_t expected_len;
    uint16_t received_len;
    uint8_t buffer[sizeof(RxPidRateAngle_t)];
} PidRxAssembler_t;
/* =========================================================
 * PACKET 1 (32 BYTE)
 * ========================================================= */

typedef struct __attribute__((packed))
{
    uint8_t  id;
    uint16_t sequence;

    int16_t  thrust_sp;

    int16_t  u_esc_fl;
    int16_t  u_esc_fr;
    int16_t  u_esc_bl;
    int16_t  u_esc_br;

    float omega_body_x;
    float omega_body_y;
    float omega_body_z;

    int16_t w_fl;
    float   battery;

    uint8_t reserved;

} Packet1_t;

/* =========================================================
 * PACKET 2 (32 BYTE)
 * ========================================================= */

typedef struct __attribute__((packed))
{
    uint8_t  id;
    uint16_t sequence;

    int16_t motor_rpm;
    int16_t distance;

    int16_t euler_x;
    int16_t euler_y;
    int16_t euler_z;

    int16_t imu_m_x;
    int16_t imu_m_y;
    int16_t imu_m_z;

    int16_t accel_body_x;
    int16_t accel_body_y;
    int16_t accel_body_z;

    int16_t w_fr;

    float baro_pressure;

    uint8_t reserved;

} Packet2_t;

/* =========================================================
 * PACKET 3 (32 BYTE)
 * ========================================================= */

typedef struct __attribute__((packed))
{
    uint8_t  id;
    uint16_t sequence;

    int16_t sp_p;
    int16_t sp_q;
    int16_t sp_r;

    int16_t int_p;
    int16_t err_p;

    int16_t int_q;
    int16_t err_q;

    int16_t int_r;
    int16_t err_r;

    int16_t tau_x;
    int16_t tau_y;
    int16_t tau_z;

    int16_t euler_offset_x;
    int16_t euler_offset_y;

    uint8_t reserved;

} Packet3_t;

/* =========================================================
 * FULL TELEMETRY FRAME (96 BYTE)
 * ========================================================= */

typedef struct __attribute__((packed))
{
    Packet1_t p1;
    Packet2_t p2;
    Packet3_t p3;

} TelemetryFrame96_t;

/* =========================================================
 * SIZE CHECK
 * ========================================================= */

_Static_assert(sizeof(Packet1_t) == 32, "Packet1 size error");
_Static_assert(sizeof(Packet2_t) == 32, "Packet2 size error");
_Static_assert(sizeof(Packet3_t) == 32, "Packet3 size error");
_Static_assert(sizeof(TelemetryFrame96_t) == 96, "Frame size error");
_Static_assert(sizeof(RxPidRateAngle_t) == 42, "RxPidRateAngle size error");
_Static_assert(sizeof(RxPidPosVel_t) == 42, "RxPidPosVel size error");
_Static_assert(sizeof(RxSettingCmd_t) == 10, "RxSettingCmd size error");

/* =========================================================
 * RF24 API
 * ========================================================= */

void NRF24_IRQHandler(void);
void NRF24_Init(void);
void NRF24_Send(void);
void NRF24_BindControlSystem(ControlSystem_t *sys);
bool NRF24_ApplyPidConfig(ControlSystem_t *sys, const uint8_t *payload, uint16_t length);

#endif