#ifndef CCU_CONSTANTS
#define CCU_CONSTANTS

/* External Includes */
#include "SharedFirmwareTypes.h"

using pin = uint8_t;
using time_us = uint32_t;


namespace CCUInterfaces
{
    /* General Interface Constants */
    constexpr uint32_t SERIAL_BAUDRATE = 115200;
    constexpr uint32_t DISPLAY_BAUDRATE = 10000000;
    constexpr uint8_t ANALOG_READ_RESOLUTION = 12;
    constexpr float BIT_RESOLUTION = 4096.0f;

    /* ADC Interface Constants */
    constexpr pin SHDN_A_PIN = 9;
    constexpr pin SHDN_B_PIN = 8;
    constexpr pin SHDN_C_PIN = 7;
    constexpr pin SHDN_D_PIN = 6;
    constexpr pin SHDN_E_PIN = 5;
    constexpr pin SHDN_F_PIN = 4;
    constexpr pin SHDN_G_PIN = 3;

    constexpr pin PROXIMITY_PILOT_PIN = 15;
    constexpr pin JUMPER_OUT_PIN = 26;
    constexpr pin CONTROL_PILOT_PIN = 27;
    constexpr pin TEENSY_240_ENABLED_PIN = 29;
    constexpr pin SCALED_24V_PIN = 33;
    constexpr pin TEENSY_240_OK_PIN = 36;

    constexpr float GLV_CONV_FACTOR = 0.106699751861F;             // 4.3K / (4.3K + 36K)
    constexpr float CONTROL_PILOT_CONV_FACTOR = 0.358695652174;    // 3.3K / (3.3K + 5.9K)
    constexpr float PROXIMITY_PILOT_CONV_FACTOR = 0.62962962963F;  // 5.1K / (5.1K + 3K)
    constexpr float JUMPER_OUT_CONV_FACTOR = 0.106699751861F;      // 4.3K / (4.3K + 36K)

    /* Display Interface Constants */
    constexpr pin LCD_CS_PIN = 39;
    constexpr pin LCD_SCK_PIN = 13;
    constexpr pin LCD_MISO_PIN = 12;
    constexpr pin LCD_MOSI_PIN = 11;
    constexpr pin LCD_RESET_PIN = 40;
    constexpr pin LCD_DC_PIN = 41;

    constexpr pin BUTTON1_READ_PIN = 16;
    constexpr pin BUTTON2_READ_PIN = 17;
    constexpr pin ENC_SWITCH_PIN = 19;
    constexpr pin ENC_B_PIN = 20;
    constexpr pin ENC_A_PIN = 21;

    /* Level2 Interface Constants */
    constexpr pin CONTROL_PWM_SENSE_PIN = 10;
    constexpr pin START_CHARGE_PIN = 32;

    /* Watchdog Interface Constants */
    constexpr pin WATCHDOG_KICK_PIN = 35;
    constexpr pin SOFTWARE_OK_PIN = 34; // Watchdog's !RESET pin
}

namespace CCUSystems
{
    constexpr float MAX_120V_CURRENT_AMP = 3.5;  // 3.5 A = 35 A in the charger CAN format
    constexpr float MAX_240V_CURRENT_AMP = 11.0; // 11 A = 110 A in the charger CAN format
}
namespace CCUConstants
{
    /* General Information */
    constexpr volt MIN_PACK_VOLTAGE = 403;
    constexpr volt MAX_PACK_VOLTAGE = 530;
    constexpr volt MIN_CELL_CUTOFF_VOLTAGE = 3.2F;
    constexpr volt MAX_CELL_CUTOFF_VOLTAGE = 4.25F;
    constexpr celsius MAX_CELL_CUTOFF_TEMP_CELSIUS = 45.0F;
    constexpr celsius MAX_BOARD_CUTOFF_TEMP_CELSIUS = 60.0F;

    /* Task Times */
    constexpr uint8_t KICK_WATCHDOG_PRIORITY = 1;
    constexpr time_us KICK_WATCHDOG_PERIOD = 1000; // 1 000 us = 1 kHz

    constexpr uint8_t SAMPLE_CAN_DATA_PRIORITY = 2;
    constexpr time_us SAMPLE_CAN_DATA_PERIOD = 2000; // 2 000 us = 500 Hz

    constexpr uint8_t ENQUEUE_ACU_CAN_DATA_PRIORITY = 3;
    constexpr time_us ENQUEUE_ACU_CAN_DATA_PERIOD = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t ENQUEUE_CHARGER_CAN_DATA_PRIORITY = 4;
    constexpr time_us ENQUEUE_CHARGER_CAN_DATA_PERIOD = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t SEND_ALL_DATA_PRIORITY = 5;
    constexpr time_us SEND_ALL_DATA_PERIOD = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t RECIEVE_ETHERNET_PRIORITY = 6;
    constexpr time_us ETHERNET_PERIOD = 20000; // 20 000 us = 50 Hz

    constexpr uint8_t SEND_ETHERNET_PRIORITY = 7;

    constexpr uint8_t LEVEL2_ENABLED_PRIORITY = 8;
    constexpr time_us LEVEL2_ENABLED_SAMPLE_PERIOD = 90000; // 90 000 us = 11.11 hz

    constexpr uint8_t READ_DIAL_PRIORITY = 9;
    constexpr time_us DIAL_PERIOD = 10000; // 20 000 us = 50 Hz

    constexpr uint8_t TICK_STATE_MACHINE_PRIORITY = 10;
    constexpr time_us TICK_STATE_MACHINE_PERIOD = 2000; // 2 000 us = 500 Hz

    constexpr uint8_t UPDATE_DISPLAY_PRIORITY = 11;
    constexpr time_us UPDATE_DISPLAY_PERIOD = 200000; // 200 000 us = 5 hz

    constexpr uint8_t REFRESH_DISPLAY_PRIORITY = 12;

    /* CAN Constants */
    const uint32_t ACU_CAN_BAUDRATE = 1000000; // CAN for ACU
    const uint32_t CHARGER_CAN_BAUDRATE = 500000; // CAN for charger
};


#endif