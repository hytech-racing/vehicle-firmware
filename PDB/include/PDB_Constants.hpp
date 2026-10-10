#ifndef PDB_CONSTANTS
#define PDB_CONSTANTS

#include "SharedFirmwareTypes.h"
#include "Pins.h"
#include "STM32_CANInterface.hpp"
#include "STM32_I2CInterface.hpp"

using pin = uint8_t;
using time_us = uint32_t;


namespace PDBInterfaces
{
    /* General Interface Constants */
    const uint8_t ANALOG_READ_RESOLUTION = 16; // STM32H7 ADC max; must match loadswitch_default_params::ADC1_RESOLUTION
    const uint32_t SERIAL_BAUDRATE = 115200;
}

namespace PDBConstants
{
    /* Task Times */
    constexpr uint8_t IDLE_SAMPLE_PRIORITY = 0;
    constexpr time_us IDLE_SAMPLE_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz

    constexpr uint8_t BUCK_UPDATE_PRIORITY = 1;
    constexpr time_us BUCK_UPDATE_PERIOD_US = 1000UL;           // 1 000 us = 1 kHz, the PG debounce resolution

    constexpr uint8_t TEMP_SENSOR_READ_PRIORITY = 2;
    constexpr time_us TEMP_SENSOR_READ_PERIOD_US = 100000UL;    // 100 000 us = 10 Hz, ADT75 needs ~60 ms per conversion

    constexpr uint8_t HOTSWAP_UPDATE_PRIORITY = 3;
    constexpr time_us HOTSWAP_UPDATE_PERIOD_US = 100000UL;      // 100 000 us = 10 Hz, telemetry + SMBA poll

    constexpr uint8_t LOAD_SWITCH_SAMPLE_PRIORITY = 4;
    constexpr time_us LOAD_SWITCH_SAMPLE_PERIOD_US = 10000UL;   // 10 000 us = 100 Hz, FLT + IMON

    constexpr uint8_t DEBUG_PRINT_PRIORITY = 10;                // Lowest: only runs when nothing else is due
    constexpr time_us DEBUG_PRINT_PERIOD_US = 250000UL;         // 250 000 us = 4 Hz

    /* Bring-up only */
    constexpr bool DEBUG_ENABLE_ALL_RAILS = true;

    /* CAN Constants */
    const uint32_t RAUX_CAN_BAUDRATE = 500000;
}

#endif
