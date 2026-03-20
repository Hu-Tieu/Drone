/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum
{
    SYSTEM_MODE_SAFETY = 0,        // CH4=0, CH5=0
    SYSTEM_MODE_FLIGHT,            // CH4=0, CH5=1
    SYSTEM_MODE_TEST,              // CH4=0, CH5=2
    SYSTEM_MODE_MANUAL,            // CH4=1, CH5=0
    SYSTEM_MODE_CALIBRATION,       // CH4=1, CH5=1
    SYSTEM_MODE_RESERVED,          // CH4=1, CH5=2
} SystemMode_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_MOTOR_Pin GPIO_PIN_1
#define ENCODER_MOTOR_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define V_BAT_Pin GPIO_PIN_4
#define V_BAT_GPIO_Port GPIOA
#define RF24_CLK_Pin GPIO_PIN_5
#define RF24_CLK_GPIO_Port GPIOA
#define RF24_MISO_Pin GPIO_PIN_6
#define RF24_MISO_GPIO_Port GPIOA
#define RF24_MOSI_Pin GPIO_PIN_7
#define RF24_MOSI_GPIO_Port GPIOA
#define MOTOR_4_Pin GPIO_PIN_2
#define MOTOR_4_GPIO_Port GPIOB
#define MOTOR_3_Pin GPIO_PIN_10
#define MOTOR_3_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOC
#define IMU_CLK_Pin GPIO_PIN_8
#define IMU_CLK_GPIO_Port GPIOA
#define VL53CX_IRQ_Pin GPIO_PIN_9
#define VL53CX_IRQ_GPIO_Port GPIOA
#define VL53CX_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define RF24_CSN_Pin GPIO_PIN_10
#define RF24_CSN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOTOR_1_Pin GPIO_PIN_15
#define MOTOR_1_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RF24_CE_Pin GPIO_PIN_4
#define RF24_CE_GPIO_Port GPIOB
#define RF24_IRQ_Pin GPIO_PIN_8
#define RF24_IRQ_GPIO_Port GPIOB
#define RF24_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define MOTOR_2_Pin GPIO_PIN_9
#define MOTOR_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#ifndef TOF_XSHUT_Pin
#define TOF_XSHUT_Pin GPIO_PIN_5
#endif

#ifndef TOF_XSHUT_GPIO_Port
#define TOF_XSHUT_GPIO_Port GPIOB
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
