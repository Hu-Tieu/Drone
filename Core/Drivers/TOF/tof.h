#ifndef DRIVERS_SENSOR_TOF_H_
#define DRIVERS_SENSOR_TOF_H_

#include <stdint.h>
#define TIMING_BUDGET           (200U)

extern long tof_last_distance_mm;
extern volatile uint8_t tof_enabled;

int32_t TOF_Init(void);
int32_t TOF_Reinitialize(void);
void TOF_distance(long *distance);

#endif /* DRIVERS_SENSOR_TOF_H_ */
