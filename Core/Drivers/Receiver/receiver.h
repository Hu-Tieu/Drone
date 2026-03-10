#ifndef __RECEIVER_H
#define __RECEIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define IBUS_FRAME_SIZE   32
#define RX_BUFFER_SIZE    128
#define IBUS_CHANNELS     14

typedef struct
{
    float channel[IBUS_CHANNELS];   // 0.0 – 1.0

} IBUS_ChannelData;

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t  dma_buffer[RX_BUFFER_SIZE];
    uint16_t old_pos;

} Receiver_HandleTypeDef;

void Receiver_Init(Receiver_HandleTypeDef *hrx,
                   UART_HandleTypeDef *huart);

bool Receiver_ProcessIBUS(Receiver_HandleTypeDef *hrx,
                          IBUS_ChannelData *ch);

#endif