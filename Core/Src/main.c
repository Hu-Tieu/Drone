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
/* ============================================================
 *                      TIMING DEBUG
 * ============================================================ */

#define ENABLE_TIMING_DEBUG  1

#if ENABLE_TIMING_DEBUG

#define BUDGET_RATE_US       1000U     // 1000 Hz
#define BUDGET_ANGLE_US      2000U     // 500 Hz
#define BUDGET_VEL_US        5000U     // 200 Hz
#define BUDGET_POS_US        20000U    // 50 Hz

static uint32_t dbg_last_print = 0;
static uint32_t dbg_violation_rate = 0;
static uint32_t dbg_violation_vel  = 0;
static uint32_t dbg_violation_pos  = 0;

static uint32_t dbg_max_rate = 0;
static uint32_t dbg_max_vel  = 0;
static uint32_t dbg_max_pos  = 0;

#endif
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
ControlSystem_t sys ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

#if ENABLE_TIMING_DEBUG
/* Enable DWT Cycle Counter */
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
#if ENABLE_TIMING_DEBUG
#define GET_US()  (DWT->CYCCNT / (SystemCoreClock / 1000000U))
#endif
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
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Receiver_HandleTypeDef receiver;
  uint32_t rpm_last_capture = 0;
  IMU_Init();
  PWM_Init();
  TOF_Init();
  Receiver_Init(&receiver, &huart4);
  ADC_Init();
  NRF24_Init();
  NRF24_BindControlSystem(&sys);
  Controller_Init(&sys);
  HAL_TIM_Base_Start_IT(&htim6);   // 1ms system tick
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    uint32_t flags;

    /* ===== Atomic snapshot ===== */
    __disable_irq();
    flags = g_flag.system;
    g_flag.system &= ~flags;
    __enable_irq();

    if (flags == 0U)
        continue;

    /* ===================== RF24 IRQ ===================== */
    if (flags & FLAG_RF24_IRQ)
    {
        NRF24_IRQHandler();
    }
            
    /* ===================== VL53CX IRQ ===================== */
    if (flags & FLAG_VL53CX_IRQ )
    {
        process_TOF_distance(&sys.sensor.distance);
    }

    /* ===================== CRITICAL SAFETY ===================== */
    if (sys.sensor.ibus.channel[2] == 1.0f &&
        sys.sensor.ibus.channel[0] == 1.0f)
    {
        PWM_ESC_Calib();
    }

    /* ===================== VBAT UPDATE ===================== */
    if (flags & FLAG_VBAT_HALF_UPDATE)
    {
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_HALF);
    }

    if (flags & FLAG_VBAT_FULL_UPDATE)
    {
        ADC_GetVbat(&sys.sensor.battery, ADC_CHANNEL_VBAT_FULL);
    }
    /* ============================================================
     *                      RATE LOOP 1000Hz
     * ============================================================ */
    static uint32_t rate_miss = 0;

    if (flags & FLAG_RATE_LOOP)
    {
        uint32_t now = GET_US();
        static uint32_t last = 0;

        if (now - last > 1100)  // lệch > 100us
            rate_miss++;

        last = now;
    #if ENABLE_TIMING_DEBUG
        uint32_t t_start = GET_US();
#endif

        Read_Gyro_Accele(&sys.sensor.imu);
        Estimator_Update(&sys.sensor, &sys.state, &sys.estimator);
        Controller_Rate_Loop(&sys.rate_ctrl, &sys.state, &sys.signals.torque_pid, &sys.signals.rate_sp);
        Motor_Mixer(&sys.signals.torque_pid,
                    sys.signals.thrust_sp,
                    &sys.signals.motor_desired);
        Motor_Inverse_Model(&sys.signals.motor_desired,
                            &sys.signals.u_esc,
                            sys.sensor.battery);

        if (sys.sensor.ibus.channel[4] == 1.0f)
        {
            ESC_Force(&sys.signals.u_esc);
        }

        ESC_Output(&sys.signals.u_esc);
        NRF24_Send();
#if ENABLE_TIMING_DEBUG
        uint32_t elapsed = GET_US() - t_start;

        if (elapsed > dbg_max_rate)
            dbg_max_rate = elapsed;

        if (elapsed > BUDGET_RATE_US)
            dbg_violation_rate++;
#endif
    }

    /* ============================================================
     *                      RPM FEEDBACK
     * ============================================================ */
    if (flags & FLAG_RPM_HALF)
    {
        Motor_RPM_Update_Half(&sys.sensor.motor_rpm);
        rpm_last_capture = HAL_GetTick();
    }

    if (flags & FLAG_RPM_FULL)
    {
        Motor_RPM_Update_Full(&sys.sensor.motor_rpm);
        rpm_last_capture = HAL_GetTick();
    }

    /* ============================================================
     *                      VELOCITY LOOP 200Hz
     * ============================================================ */
    if (flags & FLAG_VELOCITY_LOOP)
    {
#if ENABLE_TIMING_DEBUG
        uint32_t t_start = GET_US();
#endif 
  
        Read_Barometer(&sys.sensor.baro);
        Receiver_ProcessIBUS(&receiver, &sys.sensor.ibus);
        Receiver_Mapping(&sys.sensor, &sys.signals);

#if ENABLE_TIMING_DEBUG
        uint32_t elapsed = GET_US() - t_start;

        if (elapsed > dbg_max_vel)
            dbg_max_vel = elapsed;

        if (elapsed > BUDGET_VEL_US)
            dbg_violation_vel++;
#endif
    }

    /* ============================================================
     *                      POSITION LOOP 50Hz
     * ============================================================ */
    if (flags & FLAG_POSITION_LOOP)
    {
#if ENABLE_TIMING_DEBUG
        uint32_t t_start = GET_US();
#endif

        if (HAL_GetTick() - rpm_last_capture > 400)
        {
            sys.sensor.motor_rpm = 0.0f;
            rpm_last_capture = HAL_GetTick();
        }
        Read_Magnetic(&sys.sensor.imu);
   
#if ENABLE_TIMING_DEBUG
        uint32_t elapsed = GET_US() - t_start;

        if (elapsed > dbg_max_pos)
            dbg_max_pos = elapsed;

        if (elapsed > BUDGET_POS_US)
            dbg_violation_pos++;
#endif
    }

#if ENABLE_TIMING_DEBUG
/* ===================== 1 SECOND REPORT ===================== */
if (HAL_GetTick() - dbg_last_print > 1000)
{
    dbg_last_print = HAL_GetTick();

    printf("\r\n==== TIMING REPORT ====\r\n");
    printf("RATE  max: %lu us | over: %lu\r\n",
           dbg_max_rate, dbg_violation_rate);
    printf("VEL   max: %lu us | over: %lu\r\n",
           dbg_max_vel, dbg_violation_vel);
    printf("POS   max: %lu us | over: %lu\r\n",
           dbg_max_pos, dbg_violation_pos);

    printf("RATE miss: %lu\r\n", rate_miss);

    dbg_violation_rate = 0;
    dbg_violation_vel  = 0;
    dbg_violation_pos  = 0;

    dbg_max_rate = 0;
    dbg_max_vel  = 0;
    dbg_max_pos  = 0;

    rate_miss = 0;   // reset mỗi giây
}
#endif
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
