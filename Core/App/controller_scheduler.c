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



static const char *scheduler_flag_name(system_flag_t flag)
{
    switch (flag)
    {
    case FLAG_RATE_LOOP: return "RATE";
    case FLAG_ANGLE_LOOP: return "ANGLE";
    case FLAG_VELOCITY_LOOP: return "VEL";
    case FLAG_POSITION_LOOP: return "POS";
    case FLAG_LOWER_LOOP: return "LOWER";
    case FLAG_RPM_HALF: return "RPM_HALF";
    case FLAG_RPM_FULL: return "RPM_FULL";
    case FLAG_RF24_IRQ: return "RF24";
    case FLAG_VL53CX_IRQ: return "TOF";
    case FLAG_VBAT_HALF_UPDATE: return "VBAT_HALF";
    case FLAG_VBAT_FULL_UPDATE: return "VBAT_FULL";
    case FLAG_GYRO_UPDATE: return "GYRO";
    default: return "UNKNOWN";
    }
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
        }
        /* count miss when latency exceeds threshold (10us) */
        if (latency_us > 10U)
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
    uint32_t now_ms;
    uint32_t i;

    if (idx < 0)
    {
        return;
    }

    exec_us = scheduler_cycles_to_us(exec_cycles);

    printf("[TASK] %s exec=%lu us\r\n",
           scheduler_flag_name(flag),
           (unsigned long)exec_us);

    if (exec_us > g_task_worst_exec_us[(uint32_t)idx])
    {
        g_task_worst_exec_us[(uint32_t)idx] = exec_us;
    }
    if (g_task_exec_count[(uint32_t)idx] < 0xFFFFU)
    {
        g_task_exec_count[(uint32_t)idx]++;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - g_last_worst_report_ms) >= 1000U)
    {
        g_last_worst_report_ms = now_ms;
        printf("[TASK_WORST_1S]");
        for (i = 0U; i < FLAG_INDEX_COUNT; i++)
        {
            system_flag_t f = (system_flag_t)(1UL << i);
            if (g_task_exec_count[i] > 0U)
            {
                printf(" %s=%luus",
                       scheduler_flag_name(f),
                       (unsigned long)g_task_worst_exec_us[i]);
                g_task_worst_exec_us[i] = 0U;
                g_task_exec_count[i] = 0U;
            }
        }
        printf("\r\n");
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

    static uint16_t div_gyro        = 0;
    static uint16_t div_rate        = 0;
    static uint16_t div_angle       = 0;
    static uint16_t div_velocity    = 0;
    static uint16_t div_position    = 0;
    static uint16_t div_lower       = 0;
    static system_flag_t deferred_flags = 0U;
    static uint8_t deferred_rr_index = 0U;
    static const system_flag_t deferred_order[] = {
        FLAG_ANGLE_LOOP,
        FLAG_VELOCITY_LOOP,
        FLAG_POSITION_LOOP,
        FLAG_LOWER_LOOP
    };
    uint8_t critical_set = 0U;
    uint32_t i;
    
    /* ===================== GYRO UPDATE ===================== */
    if (++div_gyro>= DIV_GYRO_UPDATE)
    {
        div_gyro = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_GYRO_UPDATE);
        critical_set = 1U;
    }
    

    /* ================= RATE LOOP ================= */
    if (++div_rate >= DIV_RATE_LOOP)
    {
        div_rate = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_RATE_LOOP);
        critical_set = 1U;
    }

    /* ================= ANGLE LOOP ================= */
    if (++div_angle >= DIV_ANGLE_LOOP)
    {
        div_angle = 0;
        deferred_flags |= FLAG_ANGLE_LOOP;
    }

    /* ================= VELOCITY LOOP ================= */
    if (++div_velocity >= DIV_VELOCITY_LOOP)
    {
        div_velocity = 0;
        deferred_flags |= FLAG_VELOCITY_LOOP;
    }
    if (++div_position >= DIV_POSITION_LOOP)
    {
        div_position = 0;
        deferred_flags |= FLAG_POSITION_LOOP;
    }
    /* ================= LOWER LOOP ================= */
    if (++div_lower >= DIV_LOWER_LOOP)
    {
        div_lower = 0;
        deferred_flags |= FLAG_LOWER_LOOP;
    }

    /* Emit at most one non-critical flag per tick, and never on GYRO/RATE ticks. */
    if ((critical_set == 0U) && (deferred_flags != 0U))
    {
        for (i = 0U; i < (sizeof(deferred_order) / sizeof(deferred_order[0])); i++)
        {
            uint8_t idx = (uint8_t)((deferred_rr_index + i) % (sizeof(deferred_order) / sizeof(deferred_order[0])));
            system_flag_t flag = deferred_order[idx];

            if ((deferred_flags & flag) != 0U)
            {
                Scheduler_SetFlagWithTimestamp(flag);
                deferred_flags &= ~flag;
                deferred_rr_index = (uint8_t)((idx + 1U) % (sizeof(deferred_order) / sizeof(deferred_order[0])));
                break;
            }
        }
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
        Scheduler_SetFlagWithTimestamp(FLAG_VL53CX_IRQ);
    
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
