#ifndef DRIVERS_SENSOR_ADC_H_
#define DRIVERS_SENSOR_ADC_H_

#include <stdint.h>

/* ================= CONFIG ================= */

#define ADC_BUFFER_LENGTH     512U      // phải chia hết cho 2
#define ADC_FILTER_ALPHA      0.1f     // LPF hệ số
#define ADC_CHANNEL_VBAT_HALF 0U
#define ADC_CHANNEL_VBAT_FULL 1U
#define R_TOP                 4.59f
#define R_BOTTOM              0.99f
/* ================= API ================= */

void ADC_Init(void);
void ADC_GetVbat(float *vbat, uint8_t channel);

#endif /* DRIVERS_SENSOR_ADC_H_ */