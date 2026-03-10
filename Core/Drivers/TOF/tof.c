/*
 * motor.c
 *
 *  Created on: Feb 13, 2026
 *      Author: trinh
 */
#include "i2c.h"
#include "tof.h"
#include "custom_ranging_sensor.h"
RANGING_SENSOR_ProfileConfig_t Profile;
RANGING_SENSOR_Result_t Result;
void TOF_Init(void) {
    CUSTOM_RANGING_SENSOR_Init(CUSTOM_VL53L3CX);
    Profile.RangingProfile = RS_MULTI_TARGET_LONG_RANGE;
    Profile.TimingBudget = TIMING_BUDGET;
    Profile.Frequency = (uint32_t) 1000 / TIMING_BUDGET; // Hz
    Profile.EnableAmbient = 1;
    Profile.EnableSignal = 1;
    CUSTOM_RANGING_SENSOR_ConfigProfile(CUSTOM_VL53L3CX, &Profile);
    CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L3CX, RS_MODE_ASYNC_CONTINUOUS);
}
void process_TOF_distance(long *distance) {
    uint8_t j;
    int32_t ret = CUSTOM_RANGING_SENSOR_GetDistance(CUSTOM_VL53L3CX, &Result);
    if (ret != 0) {
        // reset_device();
        return;   // lỗi I2C → giữ -1
    }

    for (j = 0; j < Result.ZoneResult[0].NumberOfTargets; j++) {
        if (Result.ZoneResult[0].Status[j] == 0) {
        	*distance = (long) Result.ZoneResult[0].Distance[j];
            break;
        }
    }
}
