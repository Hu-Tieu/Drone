/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fmpi2c.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "controller_scheduler.h"
#include "controller_main.h"
#include "IMU/imu.h"
#include "PWM/PWM.h"
#include "RF24/rf24.h"
#include "TOF/tof.h"
#include "Battery/battery.h"
#include "Receiver/receiver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ControlSystem_t sys = {0} ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

/* Enable DWT cycle counter for scheduler delay monitoring. */
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0U;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_FMPI2C1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  Receiver_HandleTypeDef receiver;
  uint32_t rpm_last_capture = 0;
  IMU_Init();
  PWM_Init();
  // TOF_Init();
  Receiver_Init(&receiver, &huart5);
  ADC_Init();
  NRF24_Init();
  NRF24_BindControlSystem(&sys);
  Controller_Init(&sys);
  HAL_TIM_Base_Start_IT(&htim6);   // 0.625ms system tick
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
  static uint32_t pending_flags = 0U;
  static uint32_t main_loop_worst_cycles = 0U;
  static uint32_t main_loop_last_report_ms = 0U;
  static uint32_t worst_rf24_irq_cycles = 0U;
  static uint32_t worst_adc_vbat_half_cycles = 0U;
  static uint32_t worst_adc_vbat_full_cycles = 0U;
  static uint32_t worst_read_gyro_cycles = 0U;
  static uint32_t worst_estimator_gyro_cycles = 0U;
  static uint32_t worst_controller_rate_cycles = 0U;
  static uint32_t worst_motor_mixer_cycles = 0U;
  static uint32_t worst_motor_inverse_cycles = 0U;
  static uint32_t worst_esc_output_cycles = 0U;
  static uint32_t worst_rf24_send_cycles = 0U;
  static uint32_t worst_rpm_half_cycles = 0U;
  static uint32_t worst_rpm_full_cycles = 0U;
  static uint32_t worst_read_baro_cycles = 0U;
  static uint32_t worst_read_accel_cycles = 0U;
  static uint32_t worst_rate_interval_cycles = 0U;
  static uint32_t worst_angle_interval_cycles = 0U;
  static uint32_t worst_velocity_interval_cycles = 0U;
  static uint32_t worst_position_interval_cycles = 0U;
  static uint32_t last_rate_start_cycles = 0U;
  static uint32_t last_angle_start_cycles = 0U;
  static uint32_t last_velocity_start_cycles = 0U;
  static uint32_t last_position_start_cycles = 0U;
  static uint32_t last_rate_interval_cycles = 0U;
  static uint32_t last_angle_interval_cycles = 0U;
  static uint32_t last_velocity_interval_cycles = 0U;
  static uint32_t last_position_interval_cycles = 0U;
  static uint32_t worst_receiver_ibus_cycles = 0U;
  static uint32_t worst_receiver_mapping_cycles = 0U;
  static uint32_t worst_read_mag_cycles = 0U;
  static uint32_t worst_tof_distance_cycles = 0U;
  uint32_t main_loop_start_cycles = DWT->CYCCNT;
  uint8_t loop_had_work = 0U;
  uint32_t task_start_cycles;
  uint32_t task_exec_cycles;
    /* ===== Atomic snapshot ===== */
    __disable_irq();
        pending_flags |= g_flag.system;
        g_flag.system = 0U;
    __enable_irq();

    if (pending_flags == 0U)
    {
      goto MAIN_LOOP_TIMING;
    }
    loop_had_work = 1U;

    /* ===================== RF24 IRQ ===================== */
    if ((pending_flags & FLAG_RF24_IRQ) != 0U)
    {
    pending_flags &= ~FLAG_RF24_IRQ;
    Scheduler_ReportTaskDelayAndClear(FLAG_RF24_IRQ);
        task_start_cycles = DWT->CYCCNT;
        NRF24_IRQHandler();
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_rf24_irq_cycles)
        {
          worst_rf24_irq_cycles = task_exec_cycles;
        }
    }

    /* ===================== VBAT UPDATE ===================== */
    if ((pending_flags & FLAG_VBAT_HALF_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_VBAT_HALF_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_VBAT_HALF_UPDATE);
        task_start_cycles = DWT->CYCCNT;
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_HALF);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_adc_vbat_half_cycles)
        {
          worst_adc_vbat_half_cycles = task_exec_cycles;
        }
    }

    if ((pending_flags & FLAG_VBAT_FULL_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_VBAT_FULL_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_VBAT_FULL_UPDATE);
        task_start_cycles = DWT->CYCCNT;
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_FULL);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_adc_vbat_full_cycles)
        {
          worst_adc_vbat_full_cycles = task_exec_cycles;
        }
    }

    if ((pending_flags & FLAG_GYRO_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_GYRO_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_GYRO_UPDATE);
        task_start_cycles = DWT->CYCCNT;
        Read_Gyro(&sys.sensor.imu);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_read_gyro_cycles)
        {
          worst_read_gyro_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Estimator_GyroUpdate(&sys.sensor, &sys.state, &sys.estimator, &sys.gyro_lpf);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_estimator_gyro_cycles)
        {
          worst_estimator_gyro_cycles = task_exec_cycles;
        }
    }
    
    /* ============================================================
     *                      RATE LOOP 400Hz
     * ============================================================ */
        if ((pending_flags & FLAG_RATE_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_RATE_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_RATE_LOOP);
            /* rate loop inter-start interval: compute and report per-occurrence */
            {
              uint32_t now_cycles = DWT->CYCCNT;
              if (last_rate_start_cycles != 0U)
              {
                uint32_t interval = now_cycles - last_rate_start_cycles;
                last_rate_interval_cycles = interval; /* store latest interval (cycles) */
                if (interval > worst_rate_interval_cycles)
                {
                  worst_rate_interval_cycles = interval; /* keep worst for reference */
                }
              }
              last_rate_start_cycles = now_cycles;
            }
        task_start_cycles = DWT->CYCCNT;
        Controller_Rate_Loop(&sys.rate_ctrl, &sys.state, &sys.signals.torque_pid, &sys.signals.rate_sp);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_controller_rate_cycles)
        {
          worst_controller_rate_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Motor_Mixer(&sys.signals.torque_pid,
                    sys.signals.thrust_sp,
                    &sys.signals.motor_desired);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_motor_mixer_cycles)
        {
          worst_motor_mixer_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Motor_Inverse_Model(&sys.signals.motor_desired,
                            &sys.signals.u_esc,
                            sys.sensor.battery);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_motor_inverse_cycles)
        {
          worst_motor_inverse_cycles = task_exec_cycles;
        }
        if (sys.sensor.ibus.channel[4] == 0.0f)
        {
            // safety mode
            sys.signals.u_esc.w_fl = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_fr = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_bl = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_br = sys.sensor.ibus.channel[2];

            task_start_cycles = DWT->CYCCNT;
            ESC_Output(&sys.signals.u_esc);
            task_exec_cycles = DWT->CYCCNT - task_start_cycles;
            if (task_exec_cycles > worst_esc_output_cycles)
            {
              worst_esc_output_cycles = task_exec_cycles;
            }
        
        }
        else if (sys.sensor.ibus.channel[5] == 1.0f)
        {
            // normal output
            task_start_cycles = DWT->CYCCNT;
            ESC_Output(&sys.signals.u_esc);
            task_exec_cycles = DWT->CYCCNT - task_start_cycles;
            if (task_exec_cycles > worst_esc_output_cycles)
            {
              worst_esc_output_cycles = task_exec_cycles;
            }
        }
 
        task_start_cycles = DWT->CYCCNT;
        NRF24_Send();
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_rf24_send_cycles)
        {
          worst_rf24_send_cycles = task_exec_cycles;
        }
    }

    /* ============================================================
     *                      RPM FEEDBACK
     * ============================================================ */
        if ((pending_flags & FLAG_RPM_HALF) != 0U)
        {
        pending_flags &= ~FLAG_RPM_HALF;
        Scheduler_ReportTaskDelayAndClear(FLAG_RPM_HALF);
        task_start_cycles = DWT->CYCCNT;
        Motor_RPM_Update_Half(&sys.sensor.motor_rpm);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_rpm_half_cycles)
        {
          worst_rpm_half_cycles = task_exec_cycles;
        }
        rpm_last_capture = HAL_GetTick();
        }

        if ((pending_flags & FLAG_RPM_FULL) != 0U)
        {
        pending_flags &= ~FLAG_RPM_FULL;
        Scheduler_ReportTaskDelayAndClear(FLAG_RPM_FULL);
        task_start_cycles = DWT->CYCCNT;
        Motor_RPM_Update_Full(&sys.sensor.motor_rpm);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_rpm_full_cycles)
        {
          worst_rpm_full_cycles = task_exec_cycles;
        }
        rpm_last_capture = HAL_GetTick();
        }
    /* ============================================================
     *                      ANGLE LOOP 200Hz
     * ============================================================ */
        if ((pending_flags & FLAG_ANGLE_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_ANGLE_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_ANGLE_LOOP);
            /* angle loop inter-start interval: compute and report per-occurrence */
            {
              uint32_t now_cycles = DWT->CYCCNT;
              if (last_angle_start_cycles != 0U)
              {
                uint32_t interval = now_cycles - last_angle_start_cycles;
                last_angle_interval_cycles = interval;
                if (interval > worst_angle_interval_cycles)
                {
                  worst_angle_interval_cycles = interval;
                }
              }
              last_angle_start_cycles = now_cycles;
            }
            /* TODO: angle loop */
    }
    /* ============================================================
     *                      VELOCITY LOOP 100Hz
     * ============================================================ */
        if ((pending_flags & FLAG_VELOCITY_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_VELOCITY_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_VELOCITY_LOOP);
            /* velocity loop inter-start interval: compute and report per-occurrence */
            {
              uint32_t now_cycles = DWT->CYCCNT;
              if (last_velocity_start_cycles != 0U)
              {
                uint32_t interval = now_cycles - last_velocity_start_cycles;
                last_velocity_interval_cycles = interval;
                if (interval > worst_velocity_interval_cycles)
                {
                  worst_velocity_interval_cycles = interval;
                }
              }
              last_velocity_start_cycles = now_cycles;
            }
        task_start_cycles = DWT->CYCCNT;
        Read_Barometer(&sys.sensor.baro);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_read_baro_cycles)
        {
          worst_read_baro_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Read_Accel(&sys.sensor.imu);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_read_accel_cycles)
        {
          worst_read_accel_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Receiver_ProcessIBUS(&receiver, &sys.sensor.ibus);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_receiver_ibus_cycles)
        {
          worst_receiver_ibus_cycles = task_exec_cycles;
        }

        task_start_cycles = DWT->CYCCNT;
        Receiver_Mapping(&sys.sensor, &sys.signals);
        task_exec_cycles = DWT->CYCCNT - task_start_cycles;
        if (task_exec_cycles > worst_receiver_mapping_cycles)
        {
          worst_receiver_mapping_cycles = task_exec_cycles;
        }
    }

    /* ============================================================
     *                      POSITION LOOP 25Hz
     * ============================================================ */
        if ((pending_flags & FLAG_POSITION_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_POSITION_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_POSITION_LOOP);
            /* position loop inter-start interval: compute and report per-occurrence */
            {
              uint32_t now_cycles = DWT->CYCCNT;
              if (last_position_start_cycles != 0U)
              {
                uint32_t interval = now_cycles - last_position_start_cycles;
                last_position_interval_cycles = interval;
                if (interval > worst_position_interval_cycles)
                {
                  worst_position_interval_cycles = interval;
                }
              }
              last_position_start_cycles = now_cycles;
            }
        if (HAL_GetTick() - rpm_last_capture > 400)
        {
            sys.sensor.motor_rpm = 0.0f;
            rpm_last_capture = HAL_GetTick();
        }
      task_start_cycles = DWT->CYCCNT;
        Read_Magnetic(&sys.sensor.imu);
      task_exec_cycles = DWT->CYCCNT - task_start_cycles;
      if (task_exec_cycles > worst_read_mag_cycles)
      {
        worst_read_mag_cycles = task_exec_cycles;
      }
    }

        if ((pending_flags & FLAG_VL53CX_IRQ) != 0U)
        {
            pending_flags &= ~FLAG_VL53CX_IRQ;
            Scheduler_ReportTaskDelayAndClear(FLAG_VL53CX_IRQ);
            task_start_cycles = DWT->CYCCNT;
            TOF_distance(&tof_last_distance_mm);
            task_exec_cycles = DWT->CYCCNT - task_start_cycles;
            if (task_exec_cycles > worst_tof_distance_cycles)
            {
              worst_tof_distance_cycles = task_exec_cycles;
            }
            sys.sensor.distance = tof_last_distance_mm;
        }

        if ((pending_flags & FLAG_LOWER_LOOP) != 0U)
        {
            pending_flags &= ~FLAG_LOWER_LOOP;
            Scheduler_ReportTaskDelayAndClear(FLAG_LOWER_LOOP);
        }

    MAIN_LOOP_TIMING:
        if (loop_had_work != 0U)
        {
          uint32_t main_loop_exec_cycles = DWT->CYCCNT - main_loop_start_cycles;
          if (main_loop_exec_cycles > main_loop_worst_cycles)
          {
            main_loop_worst_cycles = main_loop_exec_cycles;
          }
        }

        if ((HAL_GetTick() - main_loop_last_report_ms) >= 1000U)
        {
          uint32_t cycles_per_us = (SystemCoreClock / 1000000U);
          uint32_t worst_us = 0U;
          /* some previous interval/next-delay vars removed — using scheduler latency/miss/error instead */
          uint32_t rate_latency_us = 0U;
          uint32_t angle_latency_us = 0U;
          uint32_t velocity_latency_us = 0U;
          uint32_t position_latency_us = 0U;
          uint32_t worst_rate_latency_us = 0U;
          uint32_t worst_angle_latency_us = 0U;
          uint32_t worst_velocity_latency_us = 0U;
          uint32_t worst_position_latency_us = 0U;
          uint32_t rate_miss = 0U;
          uint32_t angle_miss = 0U;
          uint32_t velocity_miss = 0U;
          uint32_t position_miss = 0U;
          uint32_t rate_err = 0U;
          uint32_t angle_err = 0U;
          uint32_t velocity_err = 0U;
          uint32_t position_err = 0U;
          uint32_t rf24_irq_us = 0U;
          uint32_t adc_vbat_half_us = 0U;
          uint32_t adc_vbat_full_us = 0U;
          uint32_t read_gyro_us = 0U;
          uint32_t estimator_gyro_us = 0U;
          uint32_t controller_rate_us = 0U;
          uint32_t motor_mixer_us = 0U;
          uint32_t motor_inverse_us = 0U;
          uint32_t esc_output_us = 0U;
          uint32_t rf24_send_us = 0U;
          uint32_t rpm_half_us = 0U;
          uint32_t rpm_full_us = 0U;
          uint32_t read_baro_us = 0U;
          uint32_t read_accel_us = 0U;
          uint32_t receiver_ibus_us = 0U;
          uint32_t receiver_mapping_us = 0U;
          uint32_t read_mag_us = 0U;
          uint32_t tof_distance_us = 0U;
          if (cycles_per_us != 0U)
          {
            worst_us = main_loop_worst_cycles / cycles_per_us;

            /* query per-flag latency (set->processed), worst latency, and counts */
            rate_latency_us = Scheduler_GetLastLatencyUs(FLAG_RATE_LOOP);
            angle_latency_us = Scheduler_GetLastLatencyUs(FLAG_ANGLE_LOOP);
            velocity_latency_us = Scheduler_GetLastLatencyUs(FLAG_VELOCITY_LOOP);
            position_latency_us = Scheduler_GetLastLatencyUs(FLAG_POSITION_LOOP);

            worst_rate_latency_us = Scheduler_PopWorstLatencyUs(FLAG_RATE_LOOP);
            worst_angle_latency_us = Scheduler_PopWorstLatencyUs(FLAG_ANGLE_LOOP);
            worst_velocity_latency_us = Scheduler_PopWorstLatencyUs(FLAG_VELOCITY_LOOP);
            worst_position_latency_us = Scheduler_PopWorstLatencyUs(FLAG_POSITION_LOOP);

            rate_miss = Scheduler_PopMissCount(FLAG_RATE_LOOP);
            angle_miss = Scheduler_PopMissCount(FLAG_ANGLE_LOOP);
            velocity_miss = Scheduler_PopMissCount(FLAG_VELOCITY_LOOP);
            position_miss = Scheduler_PopMissCount(FLAG_POSITION_LOOP);

            rate_err = Scheduler_PopErrorCount(FLAG_RATE_LOOP);
            angle_err = Scheduler_PopErrorCount(FLAG_ANGLE_LOOP);
            velocity_err = Scheduler_PopErrorCount(FLAG_VELOCITY_LOOP);
            position_err = Scheduler_PopErrorCount(FLAG_POSITION_LOOP);

            rf24_irq_us = worst_rf24_irq_cycles / cycles_per_us;
            adc_vbat_half_us = worst_adc_vbat_half_cycles / cycles_per_us;
            adc_vbat_full_us = worst_adc_vbat_full_cycles / cycles_per_us;
            read_gyro_us = worst_read_gyro_cycles / cycles_per_us;
            estimator_gyro_us = worst_estimator_gyro_cycles / cycles_per_us;
            controller_rate_us = worst_controller_rate_cycles / cycles_per_us;
            motor_mixer_us = worst_motor_mixer_cycles / cycles_per_us;
            motor_inverse_us = worst_motor_inverse_cycles / cycles_per_us;
            esc_output_us = worst_esc_output_cycles / cycles_per_us;
            rf24_send_us = worst_rf24_send_cycles / cycles_per_us;
            rpm_half_us = worst_rpm_half_cycles / cycles_per_us;
            rpm_full_us = worst_rpm_full_cycles / cycles_per_us;
            read_baro_us = worst_read_baro_cycles / cycles_per_us;
            read_accel_us = worst_read_accel_cycles / cycles_per_us;
            receiver_ibus_us = worst_receiver_ibus_cycles / cycles_per_us;
            receiver_mapping_us = worst_receiver_mapping_cycles / cycles_per_us;
            read_mag_us = worst_read_mag_cycles / cycles_per_us;
            tof_distance_us = worst_tof_distance_cycles / cycles_per_us;
          }
          printf("================== MAIN LOOP START ==================\r\n");
               printf("[PERF_1S][LOOP] worst=%luus\r\n", (unsigned long)worst_us);
               /* print per-loop latency (time set->processed), miss (>10us), and errors (re-sets while pending) */
               printf("[PERF_1S][LOOP] RATE latency=%luus miss=%lu err=%lu\r\n",
                 (unsigned long)rate_latency_us, (unsigned long)rate_miss, (unsigned long)rate_err);
               printf("[PERF_1S][LOOP] ANGLE latency=%luus miss=%lu err=%lu\r\n",
                 (unsigned long)angle_latency_us, (unsigned long)angle_miss, (unsigned long)angle_err);
               printf("[PERF_1S][LOOP] VEL latency=%luus miss=%lu err=%lu\r\n",
                 (unsigned long)velocity_latency_us, (unsigned long)velocity_miss, (unsigned long)velocity_err);
               printf("[PERF_1S][LOOP] POS latency=%luus miss=%lu err=%lu\r\n",
                 (unsigned long)position_latency_us, (unsigned long)position_miss, (unsigned long)position_err);
               printf("[PERF_1S][LOOP] worst_RATE_latency=%luus worst_ANGLE_latency=%luus worst_VEL_latency=%luus worst_POS_latency=%luus\r\n",
                 (unsigned long)worst_rate_latency_us, (unsigned long)worst_angle_latency_us,
                 (unsigned long)worst_velocity_latency_us, (unsigned long)worst_position_latency_us);

               printf("[PERF_1S][FUNC][IMU] read_gyro=%luus\r\n", (unsigned long)read_gyro_us);
               printf("[PERF_1S][FUNC][IMU] read_accel=%luus\r\n", (unsigned long)read_accel_us);
               printf("[PERF_1S][FUNC][IMU] read_mag=%luus\r\n", (unsigned long)read_mag_us);
               printf("[PERF_1S][FUNC][IMU] est_gyro=%luus\r\n", (unsigned long)estimator_gyro_us);
               printf("[PERF_1S][FUNC][IMU] read_baro=%luus\r\n", (unsigned long)read_baro_us);
               printf("[PERF_1S][FUNC][IMU] tof=%luus\r\n", (unsigned long)tof_distance_us);

               printf("[PERF_1S][FUNC][CTRL] rate=%luus\r\n", (unsigned long)controller_rate_us);
               printf("[PERF_1S][FUNC][CTRL] mixer=%luus\r\n", (unsigned long)motor_mixer_us);
               printf("[PERF_1S][FUNC][CTRL] inverse=%luus\r\n", (unsigned long)motor_inverse_us);
               printf("[PERF_1S][FUNC][CTRL] esc=%luus\r\n", (unsigned long)esc_output_us);
               printf("[PERF_1S][FUNC][CTRL] mapping=%luus\r\n", (unsigned long)receiver_mapping_us);

               printf("[PERF_1S][FUNC][IO] rf24_irq=%luus\r\n", (unsigned long)rf24_irq_us);
               printf("[PERF_1S][FUNC][IO] rf24_tx=%luus\r\n", (unsigned long)rf24_send_us);
               printf("[PERF_1S][FUNC][IO] vbat_half=%luus\r\n", (unsigned long)adc_vbat_half_us);
               printf("[PERF_1S][FUNC][IO] vbat_full=%luus\r\n", (unsigned long)adc_vbat_full_us);
               printf("[PERF_1S][FUNC][IO] ibus=%luus\r\n", (unsigned long)receiver_ibus_us);
               printf("[PERF_1S][FUNC][IO] rpm_half=%luus\r\n", (unsigned long)rpm_half_us);
               printf("[PERF_1S][FUNC][IO] rpm_full=%luus\r\n", (unsigned long)rpm_full_us);
          main_loop_worst_cycles = 0U;
          worst_rf24_irq_cycles = 0U;
          worst_adc_vbat_half_cycles = 0U;
          worst_adc_vbat_full_cycles = 0U;
          worst_read_gyro_cycles = 0U;
          worst_estimator_gyro_cycles = 0U;
          worst_controller_rate_cycles = 0U;
          worst_motor_mixer_cycles = 0U;
          worst_motor_inverse_cycles = 0U;
          worst_esc_output_cycles = 0U;
          worst_rf24_send_cycles = 0U;
          worst_rpm_half_cycles = 0U;
          worst_rpm_full_cycles = 0U;
          worst_read_baro_cycles = 0U;
          worst_read_accel_cycles = 0U;
          worst_receiver_ibus_cycles = 0U;
          worst_receiver_mapping_cycles = 0U;
          worst_read_mag_cycles = 0U;
          worst_tof_distance_cycles = 0U;
          worst_rate_interval_cycles = 0U;
          worst_angle_interval_cycles = 0U;
          worst_velocity_interval_cycles = 0U;
          worst_position_interval_cycles = 0U;
          main_loop_last_report_ms = HAL_GetTick();
        }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)ptr,
                      len,
                      HAL_MAX_DELAY);

    return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
