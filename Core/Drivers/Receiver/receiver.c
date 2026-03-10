#include "receiver.h"

/* ================= INIT ================= */

void Receiver_Init(Receiver_HandleTypeDef *hrx,
                   UART_HandleTypeDef *huart)
{
    hrx->huart   = huart;
    hrx->old_pos = 0;
    __HAL_UART_DISABLE(hrx->huart);

    /* 2. Disable DMA stream */
    HAL_DMA_Abort(hrx->huart->hdmarx);

    /* 3. Clear toàn bộ lỗi UART */
    __HAL_UART_CLEAR_OREFLAG(hrx->huart);
    __HAL_UART_CLEAR_FEFLAG(hrx->huart);
    __HAL_UART_CLEAR_NEFLAG(hrx->huart);
    __HAL_UART_CLEAR_PEFLAG(hrx->huart);

    /* 4. Flush DR register */
    volatile uint32_t tmp;
    tmp = hrx->huart->Instance->DR;
    (void)tmp;

    /* 5. Enable lại UART */
    __HAL_UART_ENABLE(hrx->huart);

    /* 6. Start lại DMA */
        HAL_UART_Receive_DMA(hrx->huart,
                            hrx->dma_buffer,
                            RX_BUFFER_SIZE);
}

/* ================= CHECKSUM ================= */

static bool IBUS_Checksum(uint8_t *frame)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < 30; i++)
        sum += frame[i];

    uint16_t checksum = 0xFFFF - sum;
    uint16_t received = frame[30] | (frame[31] << 8);

    return (checksum == received);
}

/* ================= PROCESS ================= */

bool Receiver_ProcessIBUS(Receiver_HandleTypeDef *hrx,
                          IBUS_ChannelData *ch)
{
    uint16_t pos;
    uint16_t available;

    /* DMA current write position */
    pos = RX_BUFFER_SIZE -
          __HAL_DMA_GET_COUNTER(hrx->huart->hdmarx);

    /* Calculate available bytes */
    if (pos >= hrx->old_pos)
        available = pos - hrx->old_pos;
    else
        available = RX_BUFFER_SIZE - hrx->old_pos + pos;

    if (available < IBUS_FRAME_SIZE)
        return false;

    uint8_t frame[IBUS_FRAME_SIZE];

    for (uint16_t i = 0; i < IBUS_FRAME_SIZE; i++)
    {
        frame[i] =
            hrx->dma_buffer[(hrx->old_pos + i) % RX_BUFFER_SIZE];
    }

    hrx->old_pos =
        (hrx->old_pos + IBUS_FRAME_SIZE) % RX_BUFFER_SIZE;

    /* Header check */
    if (frame[0] != 0x20 || frame[1] != 0x40)
        return false;

    /* Checksum */
    if (!IBUS_Checksum(frame))
        return false;

    /* Parse & normalize directly */
    for (uint8_t i = 0; i < IBUS_CHANNELS; i++)
    {
        uint16_t value =
            frame[2 + i*2] |
            (frame[3 + i*2] << 8);

        /* 1000–2000 → 0–1 */
        float norm = (float)(value - 1000U) * 0.001f;

        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        ch->channel[i] = norm;
    }

    return true;
}