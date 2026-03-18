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

#define SCHEDULER_MISS_THRESHOLD_US   (10U)
#define FLAG_INDEX_COUNT              (32U)

static volatile uint32_t g_flag_set_cycle[FLAG_INDEX_COUNT] = {0U};
static volatile uint16_t g_flag_resent_count[FLAG_INDEX_COUNT] = {0U};

static uint32_t scheduler_cycles_to_us(uint32_t cycles)
{
    uint32_t cycles_per_us = (SystemCoreClock / 1000000U);
    if (cycles_per_us == 0U)
    {
        return 0U;
    }

    return (cycles / cycles_per_us);
}

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

void Scheduler_ReportTaskDelayAndClear(system_flag_t flag)
{
    int32_t idx = scheduler_flag_to_index(flag);
    uint32_t now_cycles;
    uint32_t set_cycles;
    uint32_t delay_us;
    uint16_t resent_count;

    if (idx < 0)
    {
        return;
    }

    now_cycles = DWT->CYCCNT;

    __disable_irq();
    set_cycles = g_flag_set_cycle[(uint32_t)idx];
    resent_count = g_flag_resent_count[(uint32_t)idx];
    g_flag_set_cycle[(uint32_t)idx] = 0U;
    g_flag_resent_count[(uint32_t)idx] = 0U;
    __enable_irq();

    if (set_cycles != 0U)
    {
        delay_us = scheduler_cycles_to_us(now_cycles - set_cycles);
        if (delay_us > SCHEDULER_MISS_THRESHOLD_US)
        {
            printf("[SCHED_MISS] %s delay=%lu us (thr=%lu us)\r\n",
                   scheduler_flag_name(flag),
                   (unsigned long)delay_us,
                   (unsigned long)SCHEDULER_MISS_THRESHOLD_US);
        }
    }

    if (resent_count > 0U)
    {
        printf("[SCHED_ERR] %s set again before handled (%u)\r\n",
               scheduler_flag_name(flag),
               (unsigned int)resent_count);
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

    /* ================= ANGLE LOOP ================= */
    if (++div_angle >= DIV_ANGLE_LOOP)
    {
        div_angle = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_ANGLE_LOOP);
    }

    /* ================= VELOCITY LOOP ================= */
    if (++div_velocity >= DIV_VELOCITY_LOOP)
    {
        div_velocity = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_VELOCITY_LOOP);
    }
    if (++div_position >= DIV_POSITION_LOOP)
    {
        div_position = 0;
        Scheduler_SetFlagWithTimestamp(FLAG_POSITION_LOOP);
    }
    /* ================= LOWER LOOP ================= */
    if (++div_lower >= DIV_LOWER_LOOP)
    {
        div_lower = 0;
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
