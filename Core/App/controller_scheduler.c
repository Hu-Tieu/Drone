#include "controller_scheduler.h"
#include "main.h"
#include "TOF/tof.h"

volatile flag_t g_flag;

/* Phase-shift knobs (microseconds) for non-rate loops.
 * Defaults spread events to reduce same-tick collisions; users can tune at runtime.
 */
volatile uint32_t g_phase_angle_us = 10U;
volatile uint32_t g_phase_velocity_us = 1250U;
volatile uint32_t g_phase_position_us = 2500U;
volatile uint32_t g_phase_lower_us = 3750U;

#define FLAG_INDEX_COUNT              (32U)

static uint32_t scheduler_phase_us_to_tick_offset(uint32_t phase_us, uint32_t divider)
{
    uint32_t tick_us;
    uint32_t phase_ticks;

    if (divider == 0U)
    {
        return 0U;
    }

    tick_us = (1000000U / SCHEDULER_BASE_HZ);
    if (tick_us == 0U)
    {
        return 0U;
    }

    phase_ticks = (phase_us / tick_us);
    return (phase_ticks % divider);
}

/* telemetry helpers removed (unused) */

/* helper used by the public accessors below */
static int32_t scheduler_flag_to_index(system_flag_t flag)
{
    uint32_t idx;

    if ((flag == 0U) || ((flag & (flag - 1U)) != 0U))
    {
        return -1;
    }

    for (idx = 0U; idx < FLAG_INDEX_COUNT; idx++)
    {
        if ((flag & (1UL << idx)) != 0U)
        {
            return (int32_t)idx;
        }
    }

    return -1;
}

/* (Telemetry getters removed) */



static void Scheduler_SetFlagWithTimestamp(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);

    if (idx < 0)
    {
        return;
    }

    if ((g_flag.system & flag) != 0U)
    {
        return;
    }

    g_flag.system |= flag;
}

/* Task handlers and Scheduler_RunTasks are implemented in
 * Core/App/controller_scheduler_tasks.c to avoid duplicate symbols.
 */
#include "controller_scheduler_tasks.h"

void Scheduler_SetFlagFromISR(system_flag_t flag)
{
    Scheduler_SetFlagWithTimestamp(flag);
}

void Scheduler_RequestSensorReset(void)
{
    Scheduler_SetFlagWithTimestamp(FLAG_SENSOR_RESET);
}

/* Scheduler_SelectHighestPriority removed (unused) */
/* ================= SCHEDULER ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
        return;

    static uint32_t scheduler_tick = 0U;
    static uint16_t div_gyro        = 0;
    uint32_t tick;
    uint32_t lower_phase_tick;

    tick = scheduler_tick;
    scheduler_tick++;
    
    /* ===================== GYRO UPDATE ===================== */
    if (++div_gyro>= DIV_GYRO_UPDATE)
    {
        div_gyro = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_GYRO_READ_REQ);
    }
    lower_phase_tick = scheduler_phase_us_to_tick_offset(g_phase_lower_us, DIV_LOWER_LOOP);

    /* ================= LOWER LOOP (phase-shifted) ================= */
    if ((tick % DIV_LOWER_LOOP) == lower_phase_tick)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_LOWER_LOOP);
    }
}

/* ================= RPM FLAGS ================= */
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_RPM_HALF);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_RPM_FULL);
    }
}

/* ================= RF24 AND VL53CX FLAGS ================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    static uint32_t last_button_press_ms = 0U;
    static uint8_t button_seen = 0U;

    if (GPIO_Pin == USER_BUTTON_Pin)
    {
        uint32_t now = HAL_GetTick();
        if ((button_seen == 0U) || ((now - last_button_press_ms) >= 200U))
        {
            last_button_press_ms = now;
            button_seen = 1U;
            Scheduler_SetFlagWithTimestamp(FLAG_SENSOR_RESET);
        }
    }
    if(GPIO_Pin == RF24_IRQ_Pin){
                Scheduler_SetFlagWithTimestamp(FLAG_RF24_IRQ);
    }
    if (GPIO_Pin == VL53CX_IRQ_Pin)
  {
      if (tof_enabled)
      {
        Scheduler_SetFlagWithTimestamp(FLAG_VL53CX_IRQ);
      }
  }
}

/* ================= VBAT UPDATE FLAGS ================= */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_VBAT_HALF_UPDATE);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_VBAT_FULL_UPDATE);
    }
}
