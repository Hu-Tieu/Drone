#include "controller_scheduler.h"

#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "PWM/PWM.h"
#include "RF24/rf24.h"
#include "stdio.h"
#include "stm32f4xx_hal_tim.h"

volatile flag_t g_flag;
/* ================= SCHEDULER ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
        return;

    static uint16_t div_rate        = 0;
    static uint16_t div_angle       = 0;
    static uint16_t div_velocity    = 0;
    static uint16_t div_position    = 0;
    static uint16_t div_lower       = 0;
    
    /* ================= RATE LOOP ================= */
    if (++div_rate >= DIV_RATE_LOOP)
    {
        div_rate = 0;
        g_flag.system |= FLAG_RATE_LOOP;
    }

    /* ================= ANGLE LOOP ================= */
    if (++div_angle >= DIV_ANGLE_LOOP)
    {
        div_angle = 0;
        g_flag.system |= FLAG_ANGLE_LOOP;
    }

    /* ================= VELOCITY LOOP ================= */
    if (++div_velocity >= DIV_VELOCITY_LOOP)
    {
        div_velocity = 0;
        g_flag.system |= FLAG_VELOCITY_LOOP;
    }
    if (++div_position >= DIV_POSITION_LOOP)
    {
        div_position = 0;
        g_flag.system |= FLAG_POSITION_LOOP;
    }
    /* ================= LOWER LOOP ================= */
    if (++div_lower >= DIV_LOWER_LOOP)
    {
        div_lower = 0;
        g_flag.system |= FLAG_LOWER_LOOP;
    }
}

/* ================= RPM FLAGS ================= */
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        g_flag.system |= FLAG_RPM_HALF;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        g_flag.system |= FLAG_RPM_FULL;
    }
}

/* ================= RF24 AND VL53CX FLAGS ================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == RF24_IRQ_Pin){
        g_flag.system |= FLAG_RF24_IRQ;
    }
      if (GPIO_Pin == VL53CX_IRQ_Pin)
  {
    g_flag.system |= FLAG_VL53CX_IRQ;
    
  }
}

/* ================= VBAT UPDATE FLAGS ================= */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        g_flag.system |= FLAG_VBAT_HALF_UPDATE;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        g_flag.system |= FLAG_VBAT_FULL_UPDATE;
    }
}

