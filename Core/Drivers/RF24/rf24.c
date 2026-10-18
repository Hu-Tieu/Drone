#include "main.h"
#include "rf24.h"
#include "controller_scheduler.h"
#include <string.h>
extern SPI_HandleTypeDef hspi1;

static ControlSystem_t *g_rf24_sys = NULL;
static PidRxAssembler_t g_pid_rx = {0};
static uint8_t g_last_pid_code = 0;
static uint16_t g_last_pid_seq16 = 0;

uint8_t sending = 0;
uint16_t sequence = 0;

#define RF24_TX_STUCK_TIMEOUT_MS   500U

uint8_t readReg(uint8_t reg);

static uint32_t g_rf24_tx_started_at_ms = 0U;

/* =========================================================
   SPI TRANSFER
   ========================================================= */

static uint8_t spi(uint8_t data)
{
    uint8_t rx = 0xFFU;

    (void)HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 10U);

    return rx;
}

/* =========================================================
   REGISTER WRITE
   ========================================================= */
uint8_t readReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0xFFU, 0xFFU};

    tx[0] = 0x00 | (reg & 0x1F);
    tx[1] = 0xFF;

    RF24_CSN_LOW();

    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10U);

    RF24_CSN_HIGH();

    return rx[1];
}
static void writeReg(uint8_t reg,uint8_t val)
{
    RF24_CSN_LOW();

    spi(CMD_W_REGISTER | reg);
    spi(val);

    RF24_CSN_HIGH();
}

static void writeBuf(uint8_t reg,uint8_t *buf,uint8_t len)
{
    RF24_CSN_LOW();

    spi(CMD_W_REGISTER | reg);

    (void)HAL_SPI_Transmit(&hspi1, buf, len, 10U);

    RF24_CSN_HIGH();
}

/* =========================================================
   PAYLOAD
   ========================================================= */
static void readPayload(uint8_t *buf,uint8_t len)
{
    RF24_CSN_LOW();

    spi(CMD_R_RX_PAYLOAD);

    (void)HAL_SPI_Receive(&hspi1, buf, len, 10U);

    RF24_CSN_HIGH();
}
static void writePayload(uint8_t *buf,uint8_t len)
{
    RF24_CSN_LOW();

    spi(CMD_W_TX_PAYLOAD);

    (void)HAL_SPI_Transmit(&hspi1, buf, len, 10U);

    RF24_CSN_HIGH();
}

static void flushTX()
{
    RF24_CSN_LOW();

    spi(CMD_FLUSH_TX);

    RF24_CSN_HIGH();
}
static void flushRX()
{
    RF24_CSN_LOW();

    spi(CMD_FLUSH_RX);

    RF24_CSN_HIGH();
}
static void writeCmd(uint8_t cmd,uint8_t val)
{
    RF24_CSN_LOW();

    spi(cmd);
    spi(val);

    RF24_CSN_HIGH();
}

uint8_t readPayloadWidth()
{
    uint8_t w;

    RF24_CSN_LOW();
    spi(CMD_R_RX_PL_WID);
    w = spi(0xFF);
    RF24_CSN_HIGH();

    return w;
}
/* =========================================================
   INIT
   ========================================================= */

void NRF24_Init(void)
{
    RF24_CE_LOW();
    flushTX();
    flushRX();
    writeReg(REG_STATUS,0x70);
    writeReg(REG_CONFIG,0x00); // power down
    HAL_Delay(5);


    writeCmd(0x50,0x73);

    writeReg(0x1D,0x06);
    writeReg(0x1C,0x01);
    uint8_t addr[5]={'D','R','O','N','E'};

    writeBuf(REG_TX_ADDR,addr,5);
    writeBuf(REG_RX_ADDR_P0,addr,5);
    
    /* bật auto ack */

    writeReg(REG_EN_AA,0x01);

    writeReg(REG_EN_RXADDR,0x01);

    writeReg(REG_SETUP_AW,0x03);

    writeReg(REG_RF_CH,0);

    writeReg(REG_RF_SETUP,0x0E);  

    writeReg(REG_RX_PW_P0,32);

    /* auto retry */

    writeReg(REG_SETUP_RETR,0xff);           // SETUP_RETR

    writeReg(REG_STATUS,0x70);

    writeReg(REG_CONFIG,0x0E);     // PWR_UP + CRC + TX

    HAL_Delay(5);

    flushTX();

}
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}
/* =========================================================
   SEND
   ========================================================= */


static int16_t f32_to_i16(float v, float scale)
{
    float s = v * scale;
    if (s > 32767.0f) { s = 32767.0f; }
    if (s < -32768.0f) { s = -32768.0f; }
    return (int16_t)s;
}

void NRF24_BindControlSystem(ControlSystem_t *sys)
{
    g_rf24_sys = sys;
}

static void applyPid3(PID_t *x, PID_t *y, PID_t *z, const int16_t gains[3][3])
{
    x->kp = (float)gains[0][0] / PID_RX_SCALE;
    x->ki = (float)gains[0][1] / PID_RX_SCALE;
    x->kd = (float)gains[0][2] / PID_RX_SCALE;

    y->kp = (float)gains[1][0] / PID_RX_SCALE;
    y->ki = (float)gains[1][1] / PID_RX_SCALE;
    y->kd = (float)gains[1][2] / PID_RX_SCALE;

    z->kp = (float)gains[2][0] / PID_RX_SCALE;
    z->ki = (float)gains[2][1] / PID_RX_SCALE;
    z->kd = (float)gains[2][2] / PID_RX_SCALE;
}

bool NRF24_ApplyPidConfig(ControlSystem_t *sys, const uint8_t *payload, uint16_t length)
{
    if ((sys == NULL) || (payload == NULL) || (length < 2))
    {
        return false;
    }

    uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);

    if ((id == RX_PID_RATE_ANGLE_ID) && (length == sizeof(RxPidRateAngle_t)))
    {
        RxPidRateAngle_t pkt;
        int16_t rate_gains[3][3];
        int16_t angle_gains[3][3];
        memcpy(&pkt, payload, sizeof(pkt));
        memcpy(rate_gains, pkt.rate, sizeof(rate_gains));
        memcpy(angle_gains, pkt.angle, sizeof(angle_gains));

        applyPid3(&sys->rate_ctrl.p, &sys->rate_ctrl.q, &sys->rate_ctrl.r, rate_gains);
        applyPid3(&sys->angle_ctrl.roll, &sys->angle_ctrl.pitch, &sys->angle_ctrl.yaw, angle_gains);

        g_last_pid_code = 1;
        g_last_pid_seq16 = (uint16_t)(pkt.sequence & 0xFFFFu);

        return true;
    }

    if ((id == RX_PID_POS_VEL_ID) && (length == sizeof(RxPidPosVel_t)))
    {
        RxPidPosVel_t pkt;
        int16_t pos_gains[3][3];
        int16_t vel_gains[3][3];
        memcpy(&pkt, payload, sizeof(pkt));
        memcpy(pos_gains, pkt.position, sizeof(pos_gains));
        memcpy(vel_gains, pkt.velocity, sizeof(vel_gains));

        applyPid3(&sys->position_ctrl.x, &sys->position_ctrl.y, &sys->position_ctrl.z, pos_gains);
        applyPid3(&sys->velocity_ctrl.vx, &sys->velocity_ctrl.vy, &sys->velocity_ctrl.vz, vel_gains);

        g_last_pid_code = 2;
        g_last_pid_seq16 = (uint16_t)(pkt.sequence & 0xFFFFu);

        return true;
    }

    if ((id == RX_SETTING_CMD_ID) && (length == sizeof(RxSettingCmd_t)))
    {
        RxSettingCmd_t pkt;
        memcpy(&pkt, payload, sizeof(pkt));

        if (pkt.command == SETTING_CMD_RESET_ANGLE)
        {
            Controller_RequestAngleReset(sys);
            g_last_pid_code = 3;
            g_last_pid_seq16 = (uint16_t)(pkt.sequence & 0xFFFFu);
            return true;
        }

        if (pkt.command == SETTING_CMD_RESET_RATE)
        {
            Controller_RequestRateReset(sys);
            g_last_pid_code = 4;
            g_last_pid_seq16 = (uint16_t)(pkt.sequence & 0xFFFFu);
            return true;
        }

        if (pkt.command == SETTING_CMD_RESET_SENSORS)
        {
            Scheduler_RequestSensorReset();
            g_last_pid_code = 5;
            g_last_pid_seq16 = (uint16_t)(pkt.sequence & 0xFFFFu);
            return true;
        }

        return false;
    }

    return false;
}

static uint16_t pidExpectedLengthFromId(uint16_t id)
{
    if (id == RX_PID_RATE_ANGLE_ID)
    {
        return sizeof(RxPidRateAngle_t);
    }

    if (id == RX_PID_POS_VEL_ID)
    {
        return sizeof(RxPidPosVel_t);
    }

    if (id == RX_SETTING_CMD_ID)
    {
        return sizeof(RxSettingCmd_t);
    }

    return 0;
}

static void pidAssemblerReset(void)
{
    g_pid_rx.active = 0;
    g_pid_rx.id = 0;
    g_pid_rx.expected_len = 0;
    g_pid_rx.received_len = 0;
}

static bool pidAssemblerPush(const uint8_t *payload, uint8_t len, uint8_t *out_buf, uint16_t *out_len)
{
    if ((payload == NULL) || (len == 0) || (out_buf == NULL) || (out_len == NULL))
    {
        return false;
    }

    if ((len >= 2) && (!g_pid_rx.active))
    {
        uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        uint16_t expected = pidExpectedLengthFromId(id);
        uint16_t copy_len;

        if (expected == 0)
        {
            return false;
        }

        copy_len = (len > expected) ? expected : len;

        g_pid_rx.active = 1;
        g_pid_rx.id = id;
        g_pid_rx.expected_len = expected;
        g_pid_rx.received_len = copy_len;
        memcpy(g_pid_rx.buffer, payload, copy_len);

        if (g_pid_rx.received_len == g_pid_rx.expected_len)
        {
            memcpy(out_buf, g_pid_rx.buffer, g_pid_rx.expected_len);
            *out_len = g_pid_rx.expected_len;
            pidAssemblerReset();
            return true;
        }

        return false;
    }

    if (!g_pid_rx.active)
    {
        return false;
    }

    {
        uint16_t remaining = (uint16_t)(g_pid_rx.expected_len - g_pid_rx.received_len);
        uint16_t copy_len = (len > remaining) ? remaining : len;

        memcpy(&g_pid_rx.buffer[g_pid_rx.received_len], payload, copy_len);
        g_pid_rx.received_len += copy_len;
    }

    if (g_pid_rx.received_len == g_pid_rx.expected_len)
    {
        memcpy(out_buf, g_pid_rx.buffer, g_pid_rx.expected_len);
        *out_len = g_pid_rx.expected_len;
        pidAssemblerReset();
        return true;
    }

    return false;
}

static void buildTestFrame(TelemetryFrame96_t *frame, uint16_t seq)
{
    memset(frame, 0, sizeof(*frame));

    frame->p1.id = 1;
    frame->p1.sequence = seq;
    frame->p1.thrust_sp = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.thrust_sp : 0.0f, 100.0f);
    frame->p1.u_esc_fl = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.u_esc.fl : 0.0f, 1000.0f);
    frame->p1.u_esc_fr = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.u_esc.fr : 0.0f, 1000.0f);
    frame->p1.u_esc_bl = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.u_esc.bl : 0.0f, 1000.0f);
    frame->p1.u_esc_br = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.u_esc.br : 0.0f, 1000.0f);
    frame->p1.omega_body_x = g_rf24_sys ? g_rf24_sys->state.omega_body.x : 0.0f;
    frame->p1.omega_body_y = g_rf24_sys ? g_rf24_sys->state.omega_body.y : 0.0f;
    frame->p1.omega_body_z = g_rf24_sys ? g_rf24_sys->state.omega_body.z : 0.0f;
    frame->p1.w_fl = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.motor_desired.fl : 0.0f, 1.0f);
    frame->p1.battery = g_rf24_sys ? g_rf24_sys->sensor.battery : 0.0f;
    frame->p1.reserved = (uint8_t)(0xA0u | (g_last_pid_code & 0x0Fu));

    frame->p2.id = 2;
    frame->p2.sequence = seq;
    frame->p2.motor_rpm = f32_to_i16(g_rf24_sys ? g_rf24_sys->sensor.motor_rpm : 0.0f, 1.0f);
    frame->p2.distance = (int16_t)(g_rf24_sys ? g_rf24_sys->sensor.distance : 0);
    frame->p2.euler_x = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.euler.x : 0.0f, 100.0f);
    frame->p2.euler_y = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.euler.y : 0.0f, 100.0f);
    frame->p2.euler_z = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.euler.z : 0.0f, 100.0f);
    frame->p2.imu_m_x = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.mag.x : 0.0f, 100.0f);
    frame->p2.imu_m_y = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.mag.y : 0.0f, 100.0f);
    frame->p2.imu_m_z = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.mag.z : 0.0f, 100.0f);
    frame->p2.accel_body_x = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.accel_body.x : 0.0f, 100.0f);
    frame->p2.accel_body_y = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.accel_body.y : 0.0f, 100.0f);
    frame->p2.accel_body_z = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.accel_body.z : 0.0f, 100.0f);
    frame->p2.w_fr = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.motor_desired.fr : 0.0f, 1.0f);
    frame->p2.baro_pressure = g_rf24_sys ? g_rf24_sys->state.baro_altitude : 0.0f;
    frame->p2.reserved = (uint8_t)(g_last_pid_seq16 & 0xFFu);

    frame->p3.id = 3;
    frame->p3.sequence = seq;
    frame->p3.sp_p = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.rate_sp.p : 0.0f, 100.0f);
    frame->p3.sp_q = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.rate_sp.q : 0.0f, 100.0f);
    frame->p3.sp_r = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.rate_sp.r : 0.0f, 100.0f);
    frame->p3.int_p = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.p.integrator : 0.0f, 100.0f);
    frame->p3.err_p = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.p.prev_error : 0.0f, 100.0f);
    frame->p3.int_q = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.q.integrator : 0.0f, 100.0f);
    frame->p3.err_q = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.q.prev_error : 0.0f, 100.0f);
    frame->p3.int_r = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.r.integrator : 0.0f, 100.0f);
    frame->p3.err_r = f32_to_i16(g_rf24_sys ? g_rf24_sys->rate_ctrl.r.prev_error : 0.0f, 100.0f);
    frame->p3.tau_x = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.torque_pid.tau_x : 0.0f, 100.0f);
    frame->p3.tau_y = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.torque_pid.tau_y : 0.0f, 100.0f);
    frame->p3.tau_z = f32_to_i16(g_rf24_sys ? g_rf24_sys->signals.torque_pid.tau_z : 0.0f, 100.0f);
    frame->p3.euler_offset_x = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.euler_offset.x : 0.0f, 100.0f);
    frame->p3.euler_offset_y = f32_to_i16(g_rf24_sys ? g_rf24_sys->state.euler_offset.y : 0.0f, 100.0f);
    frame->p3.reserved = (uint8_t)((g_last_pid_seq16 >> 8) & 0xFFu);
}

void NRF24_Send(void)
{
    if (sending)
    {
        /* Poll TX completion as a fallback in case the EXTI flag was missed. */
        NRF24_IRQHandler();
    }

    if (sending)
    {
        if ((HAL_GetTick() - g_rf24_tx_started_at_ms) >= RF24_TX_STUCK_TIMEOUT_MS)
        {
            RF24_CE_LOW();
            flushTX();
            writeReg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
            sending = 0U;
        }
        else
        {
            return;
        }
    }

    TelemetryFrame96_t frame;
    buildTestFrame(&frame, sequence++);
    uint8_t *msg = (uint8_t *)&frame;

    RF24_CE_LOW();
    flushTX();
    flushRX();
    writeReg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

    for (int p = 0; p < 3; p++)
    {
        writePayload(&msg[p * 32], 32);
    }

    g_rf24_tx_started_at_ms = HAL_GetTick();
    sending = 1;
    RF24_CE_HIGH();
}

void NRF24_IRQHandler(void)
{
    uint8_t status = readReg(REG_STATUS);

    if (status & STATUS_MAX_RT)
    {
        writeReg(REG_STATUS, STATUS_MAX_RT);
        flushTX();
        RF24_CE_LOW();
        sending = 0;
        return;
    }

    if (status & (STATUS_RX_DR | STATUS_TX_DS))
    {
        while ((readReg(REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0)
        {
            uint8_t len = readPayloadWidth();
            if (len == 0 || len > 32)
            {
                flushRX();
                break;
            }

            uint8_t buf[32] = {0};
            readPayload(buf, len);

            if (len == sizeof(AckPayload_t))
            {
                AckPayload_t ack;
                memcpy(&ack, buf, sizeof(ack));
                (void)ack;
            }
            else
            {
                uint16_t full_len = 0;
                uint8_t full_buf[sizeof(RxPidRateAngle_t)] = {0};

                if (pidAssemblerPush(buf, len, full_buf, &full_len))
                {
                    if ((g_rf24_sys != NULL) && NRF24_ApplyPidConfig(g_rf24_sys, full_buf, full_len))
                    {
                        (void)full_len;
                    }
                }
            }
        }

        writeReg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS);
    }

    if (sending && (readReg(REG_FIFO_STATUS) & FIFO_STATUS_TX_EMPTY))
    {
        RF24_CE_LOW();
        sending = 0;
    }

}