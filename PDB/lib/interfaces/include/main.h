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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define FLT_CAMERAS_MCU_Pin GPIO_PIN_4
#define FLT_CAMERAS_MCU_GPIO_Port GPIOE
#define EN_CAMERAS_MCU_Pin GPIO_PIN_5
#define EN_CAMERAS_MCU_GPIO_Port GPIOE
#define FLT_ORIN_MCU_Pin GPIO_PIN_6
#define FLT_ORIN_MCU_GPIO_Port GPIOE
#define MCU_ID_Pin GPIO_PIN_0
#define MCU_ID_GPIO_Port GPIOC
#define SHDN_LATCH_MCU_Pin GPIO_PIN_0
#define SHDN_LATCH_MCU_GPIO_Port GPIOA
#define IMON_LIDAR_MCU_Pin GPIO_PIN_6
#define IMON_LIDAR_MCU_GPIO_Port GPIOA
#define IMON_DTI_MCU_Pin GPIO_PIN_7
#define IMON_DTI_MCU_GPIO_Port GPIOA
#define IMON_INV_COOL_MCU_Pin GPIO_PIN_4
#define IMON_INV_COOL_MCU_GPIO_Port GPIOC
#define IMON_MOTOR_COOL_MCU_Pin GPIO_PIN_5
#define IMON_MOTOR_COOL_MCU_GPIO_Port GPIOC
#define IMON_CAMERAS_MCU_Pin GPIO_PIN_0
#define IMON_CAMERAS_MCU_GPIO_Port GPIOB
#define IMON_ORIN_MCU_Pin GPIO_PIN_1
#define IMON_ORIN_MCU_GPIO_Port GPIOB
#define EN_ORIN_MCU_Pin GPIO_PIN_7
#define EN_ORIN_MCU_GPIO_Port GPIOE
#define FLT_INV_COOL_MCU_Pin GPIO_PIN_8
#define FLT_INV_COOL_MCU_GPIO_Port GPIOE
#define EN_INV_COOL_MCU_Pin GPIO_PIN_9
#define EN_INV_COOL_MCU_GPIO_Port GPIOE
#define FLT_MOTOR_COOL_MCU_Pin GPIO_PIN_10
#define FLT_MOTOR_COOL_MCU_GPIO_Port GPIOE
#define EN_MOTOR_COOL_MCU_Pin GPIO_PIN_11
#define EN_MOTOR_COOL_MCU_GPIO_Port GPIOE
#define FLT_LIDAR_MCU_Pin GPIO_PIN_12
#define FLT_LIDAR_MCU_GPIO_Port GPIOE
#define EN_LIDAR_MCU_Pin GPIO_PIN_13
#define EN_LIDAR_MCU_GPIO_Port GPIOE
#define FLT_DTI_MCU_Pin GPIO_PIN_14
#define FLT_DTI_MCU_GPIO_Port GPIOE
#define EN_DTI_MCU_Pin GPIO_PIN_15
#define EN_DTI_MCU_GPIO_Port GPIOE
#define PG_18V_ORIN_MCU_Pin GPIO_PIN_10
#define PG_18V_ORIN_MCU_GPIO_Port GPIOB
#define PG_24V_MAIN_MCU_Pin GPIO_PIN_11
#define PG_24V_MAIN_MCU_GPIO_Port GPIOB
#define PG_12V_DTI_MCU_Pin GPIO_PIN_12
#define PG_12V_DTI_MCU_GPIO_Port GPIOB
#define PG_12V_MAIN_MCU_Pin GPIO_PIN_13
#define PG_12V_MAIN_MCU_GPIO_Port GPIOB
#define PG_5V_MAIN_MCU_Pin GPIO_PIN_14
#define PG_5V_MAIN_MCU_GPIO_Port GPIOB
#define PG_3V3_MAIN_MCU_Pin GPIO_PIN_15
#define PG_3V3_MAIN_MCU_GPIO_Port GPIOB
#define TEMP_4D_ALERT_Pin GPIO_PIN_8
#define TEMP_4D_ALERT_GPIO_Port GPIOD
#define TEMP_4E_ALERT_Pin GPIO_PIN_9
#define TEMP_4E_ALERT_GPIO_Port GPIOD
#define TEMP_4F_ALERT_Pin GPIO_PIN_10
#define TEMP_4F_ALERT_GPIO_Port GPIOD
#define LIDAR_EN_MCU_Pin GPIO_PIN_8
#define LIDAR_EN_MCU_GPIO_Port GPIOC
#define DSMS_EN_MCU_Pin GPIO_PIN_9
#define DSMS_EN_MCU_GPIO_Port GPIOC
#define PDB_FAN_EN_MCU_Pin GPIO_PIN_10
#define PDB_FAN_EN_MCU_GPIO_Port GPIOC
#define EN_12V_DTI_MCU_Pin GPIO_PIN_11
#define EN_12V_DTI_MCU_GPIO_Port GPIOC
#define TEMP_48_ALERT_Pin GPIO_PIN_3
#define TEMP_48_ALERT_GPIO_Port GPIOD
#define TEMP_49_ALERT_Pin GPIO_PIN_4
#define TEMP_49_ALERT_GPIO_Port GPIOD
#define TEMP_4A_ALERT_Pin GPIO_PIN_5
#define TEMP_4A_ALERT_GPIO_Port GPIOD
#define TEMP_4B_ALERT_Pin GPIO_PIN_6
#define TEMP_4B_ALERT_GPIO_Port GPIOD
#define TEMP_4C_ALERT_Pin GPIO_PIN_7
#define TEMP_4C_ALERT_GPIO_Port GPIOD
#define EEPROM_WC_N_MCU_Pin GPIO_PIN_4
#define EEPROM_WC_N_MCU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
