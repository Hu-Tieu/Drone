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


#include "main.h" // for SystemMode_t

/* Decode channels 4 and 5 into a SystemMode_t (6 states, mapped 0..5).
 * - channel[4]: two positions ~ 0.0 (low) or 1.0 (high)
 * - channel[5]: three positions ~ 0.0 (low), 0.5 (mid), 1.0 (high)
 * Returns a SystemMode_t corresponding to (ch4,ch5) combinations.
 */
SystemMode_t Receiver_Decode_CH45(const IBUS_ChannelData *ch);

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t  dma_buffer[RX_BUFFER_SIZE];
    uint16_t old_pos;

} Receiver_HandleTypeDef;

/* Global receiver instance (moved from main.c). */
extern Receiver_HandleTypeDef g_receiver;

void Receiver_Init(Receiver_HandleTypeDef *hrx,
                   UART_HandleTypeDef *huart);

bool Receiver_ProcessIBUS(Receiver_HandleTypeDef *hrx,
                          IBUS_ChannelData *ch);

#endif