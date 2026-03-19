#include "controller_scheduler.h"

#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "PWM/PWM.h"
#include "RF24/rf24.h"
#include "TOF/tof.h"
#include "stdio.h"
#include "stm32f4xx_hal_tim.h"

volatile flag_t g_flag;

/* Phase-shift knobs (microseconds) for non-rate loops.
 * Defaults spread events to reduce same-tick collisions; users can tune at runtime.
 */
volatile uint32_t g_phase_angle_us = 10U;
volatile uint32_t g_phase_velocity_us = 1250U;
volatile uint32_t g_phase_position_us = 2500U;
volatile uint32_t g_phase_lower_us = 3750U;

#define FLAG_INDEX_COUNT              (32U)

static volatile uint32_t g_flag_set_cycle[FLAG_INDEX_COUNT] = {0U};
static volatile uint16_t g_flag_resent_count[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_task_worst_exec_us[FLAG_INDEX_COUNT] = {0U};
static uint16_t g_task_exec_count[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_last_worst_report_ms = 0U;
static uint32_t g_flag_prev_set_cycle[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_last_interval_cycles[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_worst_interval_cycles[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_last_latency_us[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_worst_latency_us[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_worst_update_cycle[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_miss_count[FLAG_INDEX_COUNT] = {0U};
static uint32_t g_flag_error_count[FLAG_INDEX_COUNT] = {0U};

static uint32_t scheduler_cycles_to_us(uint32_t cycles)
{
    uint32_t cycles_per_us = (SystemCoreClock / 1000000U);
    if (cycles_per_us == 0U)
    {
        return 0U;
    }

    return (cycles / cycles_per_us);
}

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

static uint32_t scheduler_miss_threshold_us(system_flag_t flag)
{
    switch (flag)
    {
    case FLAG_RATE_LOOP:
        return (1000000U / RATE_LOOP_HZ);
    case FLAG_ANGLE_LOOP:
        return (1000000U / ANGLE_LOOP_HZ);
    case FLAG_VELOCITY_LOOP:
        return (1000000U / VELOCITY_LOOP_HZ);
    case FLAG_POSITION_LOOP:
        return (1000000U / POSITION_LOOP_HZ);
    case FLAG_LOWER_LOOP:
        return (1000000U / LOWER_LOOP_HZ);
    default:
        /* Keep legacy threshold for non-loop flags. */
        return 40U;
    }
}

static uint32_t scheduler_expected_period_us(system_flag_t flag)
{
    switch (flag)
    {
    case FLAG_RATE_LOOP:
        return (1000000U / RATE_LOOP_HZ);
    case FLAG_ANGLE_LOOP:
        return (1000000U / ANGLE_LOOP_HZ);
    case FLAG_VELOCITY_LOOP:
        return (1000000U / VELOCITY_LOOP_HZ);
    case FLAG_POSITION_LOOP:
        return (1000000U / POSITION_LOOP_HZ);
    case FLAG_LOWER_LOOP:
        return (1000000U / LOWER_LOOP_HZ);
    default:
        return 0U;
    }
}

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

uint32_t Scheduler_GetLastIntervalCycles(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0)
        return 0U;
    return g_flag_last_interval_cycles[(uint32_t)idx];
}

uint32_t Scheduler_GetWorstIntervalCycles(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0)
        return 0U;
    return g_flag_worst_interval_cycles[(uint32_t)idx];
}

uint32_t Scheduler_GetLastLatencyUs(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0) return 0U;
    return g_flag_last_latency_us[(uint32_t)idx];
}

uint32_t Scheduler_PopWorstLatencyUs(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0) return 0U;
    uint32_t v = g_flag_worst_latency_us[(uint32_t)idx];
    g_flag_worst_latency_us[(uint32_t)idx] = 0U;
    return v;
}

uint32_t Scheduler_PopWorstUpdateCycle(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0) return 0U;
    uint32_t v = g_flag_worst_update_cycle[(uint32_t)idx];
    g_flag_worst_update_cycle[(uint32_t)idx] = 0U;
    return v;
}

uint32_t Scheduler_PopMissCount(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0) return 0U;
    uint32_t v = g_flag_miss_count[(uint32_t)idx];
    g_flag_miss_count[(uint32_t)idx] = 0U;
    return v;
}

uint32_t Scheduler_PopErrorCount(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    if (idx < 0) return 0U;
    uint32_t v = g_flag_error_count[(uint32_t)idx];
    g_flag_error_count[(uint32_t)idx] = 0U;
    return v;
}



static void Scheduler_SetFlagWithTimestamp(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);

    if (idx < 0)
    {
        return;
    }

    if ((g_flag.system & flag) != 0U)
    {
        if (g_flag_resent_count[(uint32_t)idx] < 0xFFFFU)
        {
            g_flag_resent_count[(uint32_t)idx]++;
        }
        return;
    }

    g_flag_set_cycle[(uint32_t)idx] = DWT->CYCCNT;
    g_flag.system |= flag;
}

void Scheduler_SetFlagFromISR(system_flag_t flag)
{
    Scheduler_SetFlagWithTimestamp(flag);
}

void Scheduler_ReportTaskDelayAndClear(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    uint32_t set_cycles;
    uint16_t resent_count;

    if (idx < 0)
    {
        return;
    }

    __disable_irq();
    set_cycles = g_flag_set_cycle[(uint32_t)idx];
    resent_count = g_flag_resent_count[(uint32_t)idx];
    g_flag.system &= ~flag;
    g_flag_set_cycle[(uint32_t)idx] = 0U;
    g_flag_resent_count[(uint32_t)idx] = 0U;
    __enable_irq();

    /* compute interval between consecutive set timestamps for this flag */
    if (set_cycles != 0U)
    {
        uint32_t prev = g_flag_prev_set_cycle[(uint32_t)idx];
        if (prev != 0U)
        {
            uint32_t interval = set_cycles - prev;
            g_flag_last_interval_cycles[(uint32_t)idx] = interval;
            if (interval > g_flag_worst_interval_cycles[(uint32_t)idx])
            {
                g_flag_worst_interval_cycles[(uint32_t)idx] = interval;
            }

            /* interval-based miss: count skipped periods between two consecutive set events */
            {
                uint32_t period_us = scheduler_expected_period_us(flag);
                if (period_us != 0U)
                {
                    uint32_t interval_us = scheduler_cycles_to_us(interval);
                    if (interval_us > period_us)
                    {
                        uint32_t skipped = (interval_us - 1U) / period_us;
                        g_flag_miss_count[(uint32_t)idx] += skipped;
                    }
                }
            }
        }
        g_flag_prev_set_cycle[(uint32_t)idx] = set_cycles;
    }

    /* compute latency: time from when flag was set until now (when cleared) */
    if (set_cycles != 0U)
    {
        uint32_t now_cycles = DWT->CYCCNT;
        uint32_t latency_cycles = now_cycles - set_cycles;
        uint32_t latency_us = scheduler_cycles_to_us(latency_cycles);
        g_flag_last_latency_us[(uint32_t)idx] = latency_us;
        if (latency_us > g_flag_worst_latency_us[(uint32_t)idx])
        {
            g_flag_worst_latency_us[(uint32_t)idx] = latency_us;
            g_flag_worst_update_cycle[(uint32_t)idx] = now_cycles;
        }
        /* count miss when latency exceeds loop-specific deadline */
        if (latency_us > scheduler_miss_threshold_us(flag))
        {
            g_flag_miss_count[(uint32_t)idx]++;
        }
    }

    /* if the flag was set additional times before processing, count errors */
    if (resent_count > 0U)
    {
        /* each extra set is considered an error */
        g_flag_error_count[(uint32_t)idx] += (uint32_t)resent_count;
    }
}

void Scheduler_RecordTaskExecCycles(system_flag_t flag, uint32_t exec_cycles)
{
    int32_t idx = scheduler_flag_to_index(flag);
    uint32_t exec_us;

    if (idx < 0)
    {
        return;
    }

    exec_us = scheduler_cycles_to_us(exec_cycles);
    if (exec_us > g_task_worst_exec_us[(uint32_t)idx])
    {
        g_task_worst_exec_us[(uint32_t)idx] = exec_us;
    }
    if (g_task_exec_count[(uint32_t)idx] < 0xFFFFU)
    {
        g_task_exec_count[(uint32_t)idx]++;
    }
}

system_flag_t Scheduler_SelectHighestPriority(system_flag_t ready_flags)
{
    static const system_flag_t priority_order[] = {
        FLAG_RATE_LOOP,
        FLAG_RF24_IRQ,
        FLAG_GYRO_UPDATE,
        FLAG_RPM_HALF,
        FLAG_RPM_FULL,
        FLAG_VBAT_HALF_UPDATE,
        FLAG_VBAT_FULL_UPDATE,
        FLAG_ANGLE_LOOP,
        FLAG_VELOCITY_LOOP,
        FLAG_POSITION_LOOP,
        FLAG_LOWER_LOOP,
        FLAG_VL53CX_IRQ
    };
    uint32_t i;

    for (i = 0U; i < (sizeof(priority_order) / sizeof(priority_order[0])); i++)
    {
        if ((ready_flags & priority_order[i]) != 0U)
        {
            return priority_order[i];
        }
    }

    return 0U;
}
/* ================= SCHEDULER ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
        return;

    static uint32_t scheduler_tick = 0U;
    static uint16_t div_gyro        = 0;
    static uint16_t div_rate        = 0;
    uint32_t tick;
    uint32_t angle_phase_tick;
    uint32_t velocity_phase_tick;
    uint32_t position_phase_tick;
    uint32_t lower_phase_tick;

    tick = scheduler_tick;
    scheduler_tick++;
    
    /* ===================== GYRO UPDATE ===================== */
    if (++div_gyro>= DIV_GYRO_UPDATE)
    {
        div_gyro = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_GYRO_UPDATE);
    }
    

    /* ================= RATE LOOP ================= */
    if (++div_rate >= DIV_RATE_LOOP)
    {
        div_rate = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_RATE_LOOP);
    }

    angle_phase_tick = scheduler_phase_us_to_tick_offset(g_phase_angle_us, DIV_ANGLE_LOOP);
    velocity_phase_tick = scheduler_phase_us_to_tick_offset(g_phase_velocity_us, DIV_VELOCITY_LOOP);
    position_phase_tick = scheduler_phase_us_to_tick_offset(g_phase_position_us, DIV_POSITION_LOOP);
    lower_phase_tick = scheduler_phase_us_to_tick_offset(g_phase_lower_us, DIV_LOWER_LOOP);

    /* ================= ANGLE LOOP (phase-shifted) ================= */
    if ((tick % DIV_ANGLE_LOOP) == angle_phase_tick)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_ANGLE_LOOP);
    }

    /* ================= VELOCITY LOOP (phase-shifted) ================= */
    if ((tick % DIV_VELOCITY_LOOP) == velocity_phase_tick)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_VELOCITY_LOOP);
    }

    /* ================= POSITION LOOP (phase-shifted) ================= */
    if ((tick % DIV_POSITION_LOOP) == position_phase_tick)
    {
        Scheduler_SetFlagWithTimestamp(FLAG_POSITION_LOOP);
    }

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
