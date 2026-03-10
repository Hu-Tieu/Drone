#ifndef DRIVERS_SENSOR_TOF_H_
#define DRIVERS_SENSOR_TOF_H_

#include <stdint.h>
#define TIMING_BUDGET           (100U)
void TOF_Init(void);
void process_TOF_distance(long *distance);

#endif /* DRIVERS_SENSOR_TOF_H_ */
