#include "adc.h"
#include "battery.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

static uint16_t adc_buffer[ADC_BUFFER_LENGTH];
static float vbat_filtered = 0.0f;

/* ================= INIT ================= */

void ADC_Init(void)
{
    HAL_TIM_Base_Start(&htim8);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_LENGTH);
}

/* ================= PROCESS ================= */

void ADC_GetVbat(float *vbat, uint8_t channel)
{
    if (vbat == NULL) return;
    if (channel > 1) return;

    uint32_t start = (channel == 0) ? 0 : ADC_BUFFER_LENGTH/2;
    uint32_t end   = start + ADC_BUFFER_LENGTH/2;

    uint32_t sum_vbat = 0;
    uint32_t sum_vref = 0;

    for (uint32_t i = start; i < end; i += 2)
    {
        sum_vbat += adc_buffer[i];
        sum_vref += adc_buffer[i+1];
    }

    float samples = ADC_BUFFER_LENGTH / 4.0f;

    float raw_vbat_avg = sum_vbat / samples;
    float raw_vref_avg = sum_vref / samples;

    float vdda = 3.3f * ((float)(*VREFINT_CAL_ADDR) / raw_vref_avg);

    const float divider_gain = (R_TOP + R_BOTTOM) / R_BOTTOM;
    const float adc_scale = divider_gain / 4095.0f;

    float vbat_new = raw_vbat_avg * vdda * adc_scale;

    vbat_filtered =
        (1.0f - ADC_FILTER_ALPHA) * vbat_filtered +
        ADC_FILTER_ALPHA * vbat_new;

    *vbat = vbat_filtered;
}