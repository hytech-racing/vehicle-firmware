/**
 * @brief Pin definitions for the Power Distribution Board (PDB), Rev 1.
 * @note Each schematic net name maps to its Arduino pin (PB14, PC11, ...), as assigned in STM32CubeMX
 * @note After changing pins in CubeMX, update this file from Core/Inc/main.h: GPIOx + GPIO_PIN_n -> Pxn.
*/

#ifndef PDB_PINS_H
#define PDB_PINS_H

#include <cstdint>
#include <Arduino.h>


namespace PDBPins
{
    /* --- Load switch fault inputs (active-low, open-drain from switch) --------- */
    constexpr uint32_t FLT_CAMERAS_MCU      = PE4;
    constexpr uint32_t FLT_ORIN_MCU         = PE6;
    constexpr uint32_t FLT_INV_COOL_MCU     = PE8;
    constexpr uint32_t FLT_MOTOR_COOL_MCU   = PE10;
    constexpr uint32_t FLT_LIDAR_MCU        = PE12;
    constexpr uint32_t FLT_DTI_MCU          = PE14;

    /* --- Load switch enable outputs -------------------------------------------- */
    constexpr uint32_t EN_CAMERAS_MCU       = PE5;
    constexpr uint32_t EN_ORIN_MCU          = PE7;
    constexpr uint32_t EN_INV_COOL_MCU      = PE9;
    constexpr uint32_t EN_MOTOR_COOL_MCU    = PE11;
    constexpr uint32_t EN_LIDAR_MCU         = PE13;
    constexpr uint32_t EN_DTI_MCU           = PE15;

    /* --- Buck enable outputs --------------------------------------------------- */
    constexpr uint32_t EN_12V_DTI_MCU       = PC11;
    constexpr uint32_t LIDAR_EN_MCU         = PC8;

    /* --- Other enable outputs -------------------------------------------------- */
    constexpr uint32_t DSMS_EN_MCU          = PC9;
    constexpr uint32_t PDB_FAN_EN_MCU       = PC10;

    /* --- IMON current-monitor analog inputs (ADC1) ----------------------------- */
    constexpr uint32_t IMON_LIDAR_MCU       = PA6;
    constexpr uint32_t IMON_DTI_MCU         = PA7;
    constexpr uint32_t IMON_INV_COOL_MCU    = PC4;
    constexpr uint32_t IMON_MOTOR_COOL_MCU  = PC5;
    constexpr uint32_t IMON_CAMERAS_MCU     = PB0;
    constexpr uint32_t IMON_ORIN_MCU        = PB1;

    /* --- Power-good status inputs (from bucks / hotswap) ----------------------- */
    constexpr uint32_t PG_18V_ORIN_MCU      = PB10;
    constexpr uint32_t PG_24V_MAIN_MCU      = PB11;     // Hotswap PGD
    constexpr uint32_t PG_12V_DTI_MCU       = PB12;
    constexpr uint32_t PG_12V_MAIN_MCU      = PB13;
    constexpr uint32_t PG_5V_MAIN_MCU       = PB14;
    constexpr uint32_t PG_3V3_MAIN_MCU      = PB15;

    /* --- Temperature sensor alert inputs (I2C temp sensors) -------------------- */
    constexpr uint32_t TEMP_48_ALERT        = PD3;
    constexpr uint32_t TEMP_49_ALERT        = PD4;
    constexpr uint32_t TEMP_4A_ALERT        = PD5;
    constexpr uint32_t TEMP_4B_ALERT        = PD6;
    constexpr uint32_t TEMP_4C_ALERT        = PD7;
    constexpr uint32_t TEMP_4D_ALERT        = PD8;
    constexpr uint32_t TEMP_4E_ALERT        = PD9;
    constexpr uint32_t TEMP_4F_ALERT        = PD10;

    /* --- Hotswap --------------------------------------------------------------- */
    constexpr uint32_t HOTSWAP_SMBA_MCU     = PB5;      // I2C1_SMBA, open-drain alert, active low

    /* --- Buses (set up by the shared I2C / CAN drivers) ------------------------ */
    constexpr uint32_t I2C1_SCL             = PB6;
    constexpr uint32_t I2C1_SDA             = PB7;
    constexpr uint32_t FDCAN1_RX            = PD0;
    constexpr uint32_t FDCAN1_TX            = PD1;

    /* --- Miscellaneous --------------------------------------------------------- */
    constexpr uint32_t MCU_ID               = PC0;
    constexpr uint32_t SHDN_LATCH_MCU       = PA0;
    constexpr uint32_t EEPROM_WC_N_MCU      = PB4;      // EEPROM write-control, active-low
}

#endif
