/**
 * @brief Pin definitions for the Power Distribution Board (PDB), Rev 1.
 * @note These map each signal name to its GPIO port + pin, as assigned in STM32CubeMX. 
 *       Names match the schematic net names.
 * @note For future reference, these can be copied from main.h 
*/

#ifndef PDB_PINS_H
#define PDB_PINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* --- Load switch fault inputs (active-low, open-drain from switch) --------- */
#define FLT_CAMERAS_MCU_Pin        GPIO_PIN_4
#define FLT_CAMERAS_MCU_GPIO_Port  GPIOE
#define FLT_ORIN_MCU_Pin           GPIO_PIN_6
#define FLT_ORIN_MCU_GPIO_Port     GPIOE
#define FLT_INV_COOL_MCU_Pin       GPIO_PIN_8
#define FLT_INV_COOL_MCU_GPIO_Port GPIOE
#define FLT_MOTOR_COOL_MCU_Pin     GPIO_PIN_10
#define FLT_MOTOR_COOL_MCU_GPIO_Port GPIOE
#define FLT_LIDAR_MCU_Pin          GPIO_PIN_12
#define FLT_LIDAR_MCU_GPIO_Port    GPIOE
#define FLT_DTI_MCU_Pin            GPIO_PIN_14
#define FLT_DTI_MCU_GPIO_Port      GPIOE

/* --- Load switch enable outputs -------------------------------------------- */
#define EN_CAMERAS_MCU_Pin         GPIO_PIN_5
#define EN_CAMERAS_MCU_GPIO_Port   GPIOE
#define EN_ORIN_MCU_Pin            GPIO_PIN_7
#define EN_ORIN_MCU_GPIO_Port      GPIOE
#define EN_INV_COOL_MCU_Pin        GPIO_PIN_9
#define EN_INV_COOL_MCU_GPIO_Port  GPIOE
#define EN_MOTOR_COOL_MCU_Pin      GPIO_PIN_11
#define EN_MOTOR_COOL_MCU_GPIO_Port GPIOE
#define EN_LIDAR_MCU_Pin           GPIO_PIN_13
#define EN_LIDAR_MCU_GPIO_Port     GPIOE
#define EN_DTI_MCU_Pin             GPIO_PIN_15
#define EN_DTI_MCU_GPIO_Port       GPIOE
#define EN_12V_DTI_MCU_Pin         GPIO_PIN_11
#define EN_12V_DTI_MCU_GPIO_Port   GPIOC

/* --- Other enable outputs -------------------------------------------------- */
#define LIDAR_EN_MCU_Pin           GPIO_PIN_8
#define LIDAR_EN_MCU_GPIO_Port     GPIOC
#define DSMS_EN_MCU_Pin            GPIO_PIN_9
#define DSMS_EN_MCU_GPIO_Port      GPIOC
#define PDB_FAN_EN_MCU_Pin         GPIO_PIN_10
#define PDB_FAN_EN_MCU_GPIO_Port   GPIOC

/* --- IMON current-monitor analog inputs (ADC1) ----------------------------- */
#define IMON_LIDAR_MCU_Pin         GPIO_PIN_6
#define IMON_LIDAR_MCU_GPIO_Port   GPIOA
#define IMON_DTI_MCU_Pin           GPIO_PIN_7
#define IMON_DTI_MCU_GPIO_Port     GPIOA
#define IMON_INV_COOL_MCU_Pin      GPIO_PIN_4
#define IMON_INV_COOL_MCU_GPIO_Port GPIOC
#define IMON_MOTOR_COOL_MCU_Pin    GPIO_PIN_5
#define IMON_MOTOR_COOL_MCU_GPIO_Port GPIOC
#define IMON_CAMERAS_MCU_Pin       GPIO_PIN_0
#define IMON_CAMERAS_MCU_GPIO_Port GPIOB
#define IMON_ORIN_MCU_Pin          GPIO_PIN_1
#define IMON_ORIN_MCU_GPIO_Port    GPIOB

/* --- Power-good status inputs (from bucks) --------------------------------- */
#define PG_18V_ORIN_MCU_Pin        GPIO_PIN_10
#define PG_18V_ORIN_MCU_GPIO_Port  GPIOB
#define PG_24V_MAIN_MCU_Pin        GPIO_PIN_11
#define PG_24V_MAIN_MCU_GPIO_Port  GPIOB
#define PG_12V_DTI_MCU_Pin         GPIO_PIN_12
#define PG_12V_DTI_MCU_GPIO_Port   GPIOB
#define PG_12V_MAIN_MCU_Pin        GPIO_PIN_13
#define PG_12V_MAIN_MCU_GPIO_Port  GPIOB
#define PG_5V_MAIN_MCU_Pin         GPIO_PIN_14
#define PG_5V_MAIN_MCU_GPIO_Port   GPIOB
#define PG_3V3_MAIN_MCU_Pin        GPIO_PIN_15
#define PG_3V3_MAIN_MCU_GPIO_Port  GPIOB

/* --- Temperature sensor alert inputs (I2C temp sensors) -------------------- */
#define TEMP_48_ALERT_Pin          GPIO_PIN_3
#define TEMP_48_ALERT_GPIO_Port    GPIOD
#define TEMP_49_ALERT_Pin          GPIO_PIN_4
#define TEMP_49_ALERT_GPIO_Port    GPIOD
#define TEMP_4A_ALERT_Pin          GPIO_PIN_5
#define TEMP_4A_ALERT_GPIO_Port    GPIOD
#define TEMP_4B_ALERT_Pin          GPIO_PIN_6
#define TEMP_4B_ALERT_GPIO_Port    GPIOD
#define TEMP_4C_ALERT_Pin          GPIO_PIN_7
#define TEMP_4C_ALERT_GPIO_Port    GPIOD
#define TEMP_4D_ALERT_Pin          GPIO_PIN_8
#define TEMP_4D_ALERT_GPIO_Port    GPIOD
#define TEMP_4E_ALERT_Pin          GPIO_PIN_9
#define TEMP_4E_ALERT_GPIO_Port    GPIOD
#define TEMP_4F_ALERT_Pin          GPIO_PIN_10
#define TEMP_4F_ALERT_GPIO_Port    GPIOD

/* --- Miscellaneous --------------------------------------------------------- */
#define MCU_ID_Pin                 GPIO_PIN_0
#define MCU_ID_GPIO_Port           GPIOC
#define SHDN_LATCH_MCU_Pin         GPIO_PIN_0
#define SHDN_LATCH_MCU_GPIO_Port   GPIOA
#define EEPROM_WC_N_MCU_Pin        GPIO_PIN_4   /* EEPROM write-control, active-low */
#define EEPROM_WC_N_MCU_GPIO_Port  GPIOB

#ifdef __cplusplus
}
#endif

#endif