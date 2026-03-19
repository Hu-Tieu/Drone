/*
 * tof.c
 *
 *  Created on: Feb 13, 2026
 *      Author: trinh
 */
#include "tof.h"
#include "custom_ranging_sensor.h"

long tof_last_distance_mm = 0;
volatile uint8_t tof_enabled = 1U;

RANGING_SENSOR_ProfileConfig_t Profile;
RANGING_SENSOR_Result_t Result;

static void TOF_UpdateDistanceFromResult(long *distance)
{
    uint8_t j;
    uint8_t num_targets = (uint8_t)Result.ZoneResult[0].NumberOfTargets;

    if (num_targets > RANGING_SENSOR_NB_TARGET_PER_ZONE)
    {
        num_targets = RANGING_SENSOR_NB_TARGET_PER_ZONE;
    }

    for (j = 0; j < num_targets; j++) {
        uint32_t status = Result.ZoneResult[0].Status[j];
        uint32_t dist = Result.ZoneResult[0].Distance[j];

        if (dist == 0U) {
            continue;
        }

        if ((status == 0U) || (status == 6U) || (status == 11U)) {
            tof_last_distance_mm = (long) dist;
            if (distance != NULL)
            {
                *distance = tof_last_distance_mm;
            }
            return;
        }

        /* Keep last good sample when status is invalid to avoid jitter spikes. */
    }
}

int32_t TOF_Init(void) {
    int32_t ret;
    
    ret = CUSTOM_RANGING_SENSOR_Init(CUSTOM_VL53L3CX);
    if (ret != BSP_ERROR_NONE)
    {
        return ret;
    }
    
    Profile.RangingProfile = RS_MULTI_TARGET_LONG_RANGE;
    Profile.TimingBudget = TIMING_BUDGET;
    Profile.Frequency = (uint32_t) 1000 / TIMING_BUDGET; // Hz
    Profile.EnableAmbient = 1;
    Profile.EnableSignal = 1;
    
    ret = CUSTOM_RANGING_SENSOR_ConfigProfile(CUSTOM_VL53L3CX, &Profile);
    if (ret != BSP_ERROR_NONE)
    {
        return ret;
    }
    
    ret = CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L3CX, RS_MODE_ASYNC_CONTINUOUS);
    return ret;
}

void TOF_distance(long *distance)
{
    if (CUSTOM_RANGING_SENSOR_GetDistance(CUSTOM_VL53L3CX, &Result) == BSP_ERROR_NONE)
    {
        TOF_UpdateDistanceFromResult(distance);
    }
}
