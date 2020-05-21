/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT11.h"
#include "DS3231.h"
#include "APDS9960.h"
#include "MPU6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


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
#define TFT_T_IRQ_Pin GPIO_PIN_13
#define TFT_T_IRQ_GPIO_Port GPIOC
#define TFT_T_DO_Pin GPIO_PIN_14
#define TFT_T_DO_GPIO_Port GPIOC
#define TFT_T_DIN_Pin GPIO_PIN_15
#define TFT_T_DIN_GPIO_Port GPIOC
#define TFT_T_CS_Pin GPIO_PIN_0
#define TFT_T_CS_GPIO_Port GPIOA
#define TFT_T_CLK_Pin GPIO_PIN_1
#define TFT_T_CLK_GPIO_Port GPIOA
#define TFT_DC_Pin GPIO_PIN_2
#define TFT_DC_GPIO_Port GPIOA
#define TFT_RESET_Pin GPIO_PIN_3
#define TFT_RESET_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOA
#define TFT_SCK_Pin GPIO_PIN_5
#define TFT_SCK_GPIO_Port GPIOA
#define TFT_SDO_Pin GPIO_PIN_6
#define TFT_SDO_GPIO_Port GPIOA
#define TFT_SDI_Pin GPIO_PIN_7
#define TFT_SDI_GPIO_Port GPIOA
#define TFT_LED_Pin GPIO_PIN_0
#define TFT_LED_GPIO_Port GPIOB
#define EC11_B_Pin GPIO_PIN_1
#define EC11_B_GPIO_Port GPIOB
#define EC11_A_Pin GPIO_PIN_10
#define EC11_A_GPIO_Port GPIOB
#define EC11_A_EXTI_IRQn EXTI15_10_IRQn
#define DHT11_SDA_Pin GPIO_PIN_11
#define DHT11_SDA_GPIO_Port GPIOB
#define RTC_SDA_Pin GPIO_PIN_12
#define RTC_SDA_GPIO_Port GPIOB
#define RTC_SCL_Pin GPIO_PIN_13
#define RTC_SCL_GPIO_Port GPIOB
#define RTC_SQW_Pin GPIO_PIN_14
#define RTC_SQW_GPIO_Port GPIOB
#define RTC_32K_Pin GPIO_PIN_15
#define RTC_32K_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define APDS_SCL_Pin GPIO_PIN_11
#define APDS_SCL_GPIO_Port GPIOA
#define APDS_SDA_Pin GPIO_PIN_12
#define APDS_SDA_GPIO_Port GPIOA
#define APDS_INT_Pin GPIO_PIN_15
#define APDS_INT_GPIO_Port GPIOA
#define APDS_INT_EXTI_IRQn EXTI15_10_IRQn
#define EC11_KEY_Pin GPIO_PIN_3
#define EC11_KEY_GPIO_Port GPIOB
#define EC11_KEY_EXTI_IRQn EXTI3_IRQn
#define MPU6050_SCL_Pin GPIO_PIN_4
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_5
#define MPU6050_SDA_GPIO_Port GPIOB
#define MPU6050_ADO_Pin GPIO_PIN_8
#define MPU6050_ADO_GPIO_Port GPIOB
#define MPU6050_INT_Pin GPIO_PIN_9
#define MPU6050_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
