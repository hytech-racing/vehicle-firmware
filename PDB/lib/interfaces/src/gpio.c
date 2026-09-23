/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_CAMERAS_MCU_Pin|EN_ORIN_MCU_Pin|EN_INV_COOL_MCU_Pin|EN_MOTOR_COOL_MCU_Pin
                          |EN_LIDAR_MCU_Pin|EN_DTI_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LIDAR_EN_MCU_Pin|DSMS_EN_MCU_Pin|PDB_FAN_EN_MCU_Pin|EN_12V_DTI_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEPROM_WC_N_MCU_GPIO_Port, EEPROM_WC_N_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLT_CAMERAS_MCU_Pin FLT_ORIN_MCU_Pin FLT_INV_COOL_MCU_Pin FLT_MOTOR_COOL_MCU_Pin
                           FLT_LIDAR_MCU_Pin FLT_DTI_MCU_Pin */
  GPIO_InitStruct.Pin = FLT_CAMERAS_MCU_Pin|FLT_ORIN_MCU_Pin|FLT_INV_COOL_MCU_Pin|FLT_MOTOR_COOL_MCU_Pin
                          |FLT_LIDAR_MCU_Pin|FLT_DTI_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_CAMERAS_MCU_Pin EN_ORIN_MCU_Pin EN_INV_COOL_MCU_Pin EN_MOTOR_COOL_MCU_Pin
                           EN_LIDAR_MCU_Pin EN_DTI_MCU_Pin */
  GPIO_InitStruct.Pin = EN_CAMERAS_MCU_Pin|EN_ORIN_MCU_Pin|EN_INV_COOL_MCU_Pin|EN_MOTOR_COOL_MCU_Pin
                          |EN_LIDAR_MCU_Pin|EN_DTI_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PG_18V_ORIN_MCU_Pin PG_24V_MAIN_MCU_Pin PG_12V_DTI_MCU_Pin PG_12V_MAIN_MCU_Pin
                           PG_5V_MAIN_MCU_Pin PG_3V3_MAIN_MCU_Pin */
  GPIO_InitStruct.Pin = PG_18V_ORIN_MCU_Pin|PG_24V_MAIN_MCU_Pin|PG_12V_DTI_MCU_Pin|PG_12V_MAIN_MCU_Pin
                          |PG_5V_MAIN_MCU_Pin|PG_3V3_MAIN_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMP_4D_ALERT_Pin TEMP_4E_ALERT_Pin TEMP_4F_ALERT_Pin TEMP_48_ALERT_Pin
                           TEMP_49_ALERT_Pin TEMP_4A_ALERT_Pin TEMP_4B_ALERT_Pin TEMP_4C_ALERT_Pin */
  GPIO_InitStruct.Pin = TEMP_4D_ALERT_Pin|TEMP_4E_ALERT_Pin|TEMP_4F_ALERT_Pin|TEMP_48_ALERT_Pin
                          |TEMP_49_ALERT_Pin|TEMP_4A_ALERT_Pin|TEMP_4B_ALERT_Pin|TEMP_4C_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LIDAR_EN_MCU_Pin DSMS_EN_MCU_Pin PDB_FAN_EN_MCU_Pin EN_12V_DTI_MCU_Pin */
  GPIO_InitStruct.Pin = LIDAR_EN_MCU_Pin|DSMS_EN_MCU_Pin|PDB_FAN_EN_MCU_Pin|EN_12V_DTI_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_WC_N_MCU_Pin */
  GPIO_InitStruct.Pin = EEPROM_WC_N_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_WC_N_MCU_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
