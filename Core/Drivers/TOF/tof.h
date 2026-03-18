#ifndef DRIVERS_SENSOR_TOF_H_
#define DRIVERS_SENSOR_TOF_H_

#include <stdint.h>
#define TIMING_BUDGET           (500U)

extern long tof_last_distance_mm;

void TOF_Init(void);
void TOF_distance(long *distance);

#endif /* DRIVERS_SENSOR_TOF_H_ */
