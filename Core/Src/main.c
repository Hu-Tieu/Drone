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
#include "string.h"
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
#define PERF_LOOP_WORST_LOG_PERIOD_MS 1000U

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
  if (tof_enabled)
  {
    TOF_Init();
  }
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
  static uint32_t main_loop_last_report_ms = 0U;
  static uint32_t main_loop_worst_cycles = 0U;
  uint32_t main_loop_start_cycles = DWT->CYCCNT;
    /* ===== Atomic snapshot =====
     * Do not clear g_flag.system here.
     * Each task flag is cleared atomically inside Scheduler_ReportTaskDelayAndClear(),
     * so set timestamp and clear happen consistently for latency accounting.
     */
    __disable_irq();
      pending_flags |= g_flag.system;
    __enable_irq();

    if (pending_flags == 0U)
    {
      goto MAIN_LOOP_REPORT;
    }

    /* ===================== RF24 IRQ ===================== */
    if ((pending_flags & FLAG_RF24_IRQ) != 0U)
    {
    pending_flags &= ~FLAG_RF24_IRQ;
    Scheduler_ReportTaskDelayAndClear(FLAG_RF24_IRQ);
        NRF24_IRQHandler();
    }

    /* ===================== VBAT UPDATE ===================== */
    if ((pending_flags & FLAG_VBAT_HALF_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_VBAT_HALF_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_VBAT_HALF_UPDATE);
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_HALF);
    }

    if ((pending_flags & FLAG_VBAT_FULL_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_VBAT_FULL_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_VBAT_FULL_UPDATE);
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_FULL);
    }

    if ((pending_flags & FLAG_GYRO_UPDATE) != 0U)
    {
    pending_flags &= ~FLAG_GYRO_UPDATE;
    Scheduler_ReportTaskDelayAndClear(FLAG_GYRO_UPDATE);
        Read_Gyro(&sys.sensor.imu);
        Estimator_GyroUpdate(&sys.sensor, &sys.state, &sys.estimator, &sys.gyro_lpf);
    }
    
    /* ============================================================
     *                      RATE LOOP 400Hz
     * ============================================================ */
        if ((pending_flags & FLAG_RATE_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_RATE_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_RATE_LOOP);
        Controller_Rate_Loop(&sys.rate_ctrl, &sys.state, &sys.signals.torque_pid, &sys.signals.rate_sp);
        Motor_Mixer(&sys.signals.torque_pid,
                    sys.signals.thrust_sp,
                    &sys.signals.motor_desired);
        Motor_Inverse_Model(&sys.signals.motor_desired,
                            &sys.signals.u_esc,
                            sys.sensor.battery);
        if (sys.sensor.ibus.channel[4] == 0.0f)
        {
            // safety mode
            sys.signals.u_esc.w_fl = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_fr = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_bl = sys.sensor.ibus.channel[2];
            sys.signals.u_esc.w_br = sys.sensor.ibus.channel[2];

            ESC_Output(&sys.signals.u_esc);
        }
        else if (sys.sensor.ibus.channel[5] == 1.0f)
        {
            // normal output
            ESC_Output(&sys.signals.u_esc);
        }
 
        NRF24_Send();
    }

    /* ============================================================
     *                      RPM FEEDBACK
     * ============================================================ */
        if ((pending_flags & FLAG_RPM_HALF) != 0U)
        {
        pending_flags &= ~FLAG_RPM_HALF;
        Scheduler_ReportTaskDelayAndClear(FLAG_RPM_HALF);
        Motor_RPM_Update_Half(&sys.sensor.motor_rpm);
        rpm_last_capture = HAL_GetTick();
        }

        if ((pending_flags & FLAG_RPM_FULL) != 0U)
        {
        pending_flags &= ~FLAG_RPM_FULL;
        Scheduler_ReportTaskDelayAndClear(FLAG_RPM_FULL);
        Motor_RPM_Update_Full(&sys.sensor.motor_rpm);
        rpm_last_capture = HAL_GetTick();
        }
    /* ============================================================
     *                      ANGLE LOOP 200Hz
     * ============================================================ */
        if ((pending_flags & FLAG_ANGLE_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_ANGLE_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_ANGLE_LOOP);
            /* TODO: angle loop */
    }
    /* ============================================================
     *                      VELOCITY LOOP 100Hz
     * ============================================================ */
        if ((pending_flags & FLAG_VELOCITY_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_VELOCITY_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_VELOCITY_LOOP);
        Read_Barometer(&sys.sensor.baro);
        Read_Accel(&sys.sensor.imu);
        Receiver_ProcessIBUS(&receiver, &sys.sensor.ibus);
        Receiver_Mapping(&sys.sensor, &sys.signals);
    }

    /* ============================================================
     *                      POSITION LOOP 25Hz
     * ============================================================ */
        if ((pending_flags & FLAG_POSITION_LOOP) != 0U)
    {
        pending_flags &= ~FLAG_POSITION_LOOP;
        Scheduler_ReportTaskDelayAndClear(FLAG_POSITION_LOOP);
        if (HAL_GetTick() - rpm_last_capture > 400)
        {
            sys.sensor.motor_rpm = 0.0f;
            rpm_last_capture = HAL_GetTick();
        }
        Read_Magnetic(&sys.sensor.imu);
    }

        if (tof_enabled)
        {
            if ((pending_flags & FLAG_VL53CX_IRQ) != 0U)
            {
                pending_flags &= ~FLAG_VL53CX_IRQ;
                Scheduler_ReportTaskDelayAndClear(FLAG_VL53CX_IRQ);
                TOF_distance(&tof_last_distance_mm);
                sys.sensor.distance = tof_last_distance_mm;
            }
        }

        if ((pending_flags & FLAG_LOWER_LOOP) != 0U)
        {
            pending_flags &= ~FLAG_LOWER_LOOP;
            Scheduler_ReportTaskDelayAndClear(FLAG_LOWER_LOOP);
        }

    MAIN_LOOP_REPORT:
        {
          uint32_t main_loop_exec_cycles = DWT->CYCCNT - main_loop_start_cycles;
          if (main_loop_exec_cycles > main_loop_worst_cycles)
          {
            main_loop_worst_cycles = main_loop_exec_cycles;
          }
        }

        if ((HAL_GetTick() - main_loop_last_report_ms) >= PERF_LOOP_WORST_LOG_PERIOD_MS)
        {
          uint32_t cycles_per_us = (SystemCoreClock / 1000000U);
          if (cycles_per_us != 0U)
          {
            printf("[LOOP_WORST_MAIN] WHILE=%luus\r\n",
              (unsigned long)(main_loop_worst_cycles / cycles_per_us));
          }
          main_loop_worst_cycles = 0U;
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
/* Non-blocking UART2 logger: printf enqueues bytes, DMA sends in background. */
#define UART2_LOG_TX_BUFFER_SIZE 4096U

static uint8_t uart2_log_tx_buf[UART2_LOG_TX_BUFFER_SIZE];
static volatile uint16_t uart2_log_tx_head = 0U;
static volatile uint16_t uart2_log_tx_tail = 0U;
static volatile uint16_t uart2_log_dma_len = 0U;
static volatile uint8_t uart2_log_dma_busy = 0U;

static void UART2_LogKickDma(void)
{
  uint16_t tail;
  uint16_t head;
  uint16_t len;

  __disable_irq();
  if (uart2_log_dma_busy != 0U)
  {
    __enable_irq();
    return;
  }

  tail = uart2_log_tx_tail;
  head = uart2_log_tx_head;
  if (head == tail)
  {
    __enable_irq();
    return;
  }

  if (head > tail)
  {
    len = (uint16_t)(head - tail);
  }
  else
  {
    len = (uint16_t)(UART2_LOG_TX_BUFFER_SIZE - tail);
  }

  uart2_log_dma_len = len;
  uart2_log_dma_busy = 1U;
  __enable_irq();

  if (HAL_UART_Transmit_DMA(&huart2, &uart2_log_tx_buf[tail], len) != HAL_OK)
  {
    __disable_irq();
    uart2_log_dma_busy = 0U;
    uart2_log_dma_len = 0U;
    __enable_irq();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __disable_irq();
    uart2_log_tx_tail = (uint16_t)((uart2_log_tx_tail + uart2_log_dma_len) % UART2_LOG_TX_BUFFER_SIZE);
    uart2_log_dma_len = 0U;
    uart2_log_dma_busy = 0U;
    __enable_irq();

    UART2_LogKickDma();
  }
}

int _write(int file, char *ptr, int len)
{
  int written = 0;
  (void)file;

  if ((ptr == NULL) || (len <= 0))
  {
    return 0;
  }

  __disable_irq();
  while (written < len)
  {
    uint16_t next = (uint16_t)((uart2_log_tx_head + 1U) % UART2_LOG_TX_BUFFER_SIZE);
    if (next == uart2_log_tx_tail)
    {
      break;
    }

    uart2_log_tx_buf[uart2_log_tx_head] = (uint8_t)ptr[written];
    uart2_log_tx_head = next;
    written++;
  }
  __enable_irq();

  UART2_LogKickDma();

  return written;
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
