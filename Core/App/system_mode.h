#ifndef SYSTEM_MODE_H
#define SYSTEM_MODE_H

#include <stdint.h>

typedef enum SystemMode_t
{
    SYSTEM_MODE_SAFETY = 0,        // CH4=0, CH5=0
    SYSTEM_MODE_SAFETY1,           // CH4=0, CH5=1
    SYSTEM_MODE_RESERVED,          // CH4=0, CH5=2
    SYSTEM_MODE_TEST,              // CH4=1, CH5=0
    SYSTEM_MODE_IDLE,              // CH4=1, CH5=1
    SYSTEM_MODE_FLIGHT,            // CH4=1, CH5=2
} SystemMode_t;

typedef struct {
    uint8_t enable_rate;
    uint8_t enable_angle;
    uint8_t enable_velocity;
    uint8_t enable_position;
    uint8_t enable_motor_output;
} ModeConfig_t;

/* mode_table defined in controller_scheduler_tasks.c */
extern const ModeConfig_t mode_table[];

#endif // SYSTEM_MODE_H
