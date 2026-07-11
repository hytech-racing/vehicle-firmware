#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS

/* External Includes */
#include "SharedFirmwareTypes.h"

using pin = uint8_t;
using degree  = float;
using time_us = uint32_t;

namespace VCFInterfaces
{
    /* Serial */
    constexpr size_t SERIAL_BAUDRATE = 115200;

    /* IIR Filter Alphas */
    constexpr float LOADCELL_IIR_FILTER_ALPHA = 0.01f;

    /* ADC chip selects */
    constexpr int ADC0_CS = 10; // MCP3208 — steering, sus pots, load cells
    constexpr int ADC1_CS = 38; // MCP3208 — pedal position sensors

    /* ADC Versions*/
    /* Channels on ADC_0 */
    // constexpr int UNUSED_CHANNEL         = 0;
    constexpr int PEDAL_REF_2V5_CHANNEL     = 1;
    constexpr int STEERING_1_CHANNEL        = 2;
    constexpr int STEERING_2_CHANNEL        = 3;
    constexpr int ACCEL_1_CHANNEL           = 4;
    constexpr int ACCEL_2_CHANNEL           = 5;
    constexpr int BRAKE_1_CHANNEL           = 6;
    constexpr int BRAKE_2_CHANNEL           = 7;

    /* Channels on ADC_1 */
    constexpr int SHDN_H_CHANNEL                = 0;
    constexpr int SHDN_D_CHANNEL                = 1;
    constexpr int FL_LOADCELL_CHANNEL           = 2;
    constexpr int FR_LOADCELL_CHANNEL           = 3;
    constexpr int FR_SUS_POT_CHANNEL            = 4;
    constexpr int FL_SUS_POT_CHANNEL            = 5;
    constexpr int BRAKE_PRESSURE_FRONT_CHANNEL  = 6;
    constexpr int BRAKE_PRESSURE_REAR_CHANNEL   = 7;

    /* ADC scaling and offsets */
    constexpr float PEDAL_REF_2V5_SCALE    = 1.0f;
    constexpr float PEDAL_REF_2V5_OFFSET   = 0.0f;

    constexpr float STEERING_1_SCALE       = 1.0f; // TODO: calibrate
    constexpr float STEERING_1_OFFSET      = 0.0f;
    constexpr float STEERING_2_SCALE       = 1.0f; // TODO: calibrate
    constexpr float STEERING_2_OFFSET      = 0.0f;

    constexpr float ACCEL_1_SCALE          = 1.0f; // TODO: calibrate
    constexpr float ACCEL_1_OFFSET         = 0.0f;
    constexpr float ACCEL_2_SCALE          = 1.0f;
    constexpr float ACCEL_2_OFFSET         = 0.0f;

    constexpr float BRAKE_1_SCALE          = 1.0f;
    constexpr float BRAKE_1_OFFSET         = 0.0f;
    constexpr float BRAKE_2_SCALE          = 1.0f;
    constexpr float BRAKE_2_OFFSET         = 0.0f;

    constexpr float SHDN_D_SCALE           = 0.00697841165926f;
    constexpr float SHDN_D_OFFSET          = 0.0f;
    constexpr float SHDN_H_SCALE           = 0.00697841165926f; // ADC read → shutdown voltage
    constexpr float SHDN_H_OFFSET          = 0.0f;

    constexpr float FL_LOADCELL_SCALE      = 1.0f; // TODO: calibrate
    constexpr float FL_LOADCELL_OFFSET     = 0.0f;
    constexpr float FR_LOADCELL_SCALE      = 1.0f; // TODO: calibrate
    constexpr float FR_LOADCELL_OFFSET     = 0.0f;

    constexpr float FR_SUS_POT_SCALE       = 0.01396f; // mm between mounting bolts
    constexpr float FR_SUS_POT_OFFSET      = 150.8f;
    constexpr float FL_SUS_POT_SCALE       = 0.01396f;
    constexpr float FL_SUS_POT_OFFSET      = 150.8f;

    constexpr float BRAKE_PRESSURE_FRONT_SCALE  = 1.0f;
    constexpr float BRAKE_PRESSURE_FRONT_OFFSET = 0.0f;
    constexpr float BRAKE_PRESSURE_REAR_SCALE   = 1.0f;
    constexpr float BRAKE_PRESSURE_REAR_OFFSET  = 0.0f;

    /* Not on Schematic
    // constexpr int BTN_DIM_READ = 28; // Currently used for steering system recalibration TODO: change pin
    // constexpr int BTN_PRESET_READ = 31;
    // constexpr int BTN_MODE_READ = 27; // USED TO BE 26.
    */

    /* Dashboard GPIO */
    constexpr int BRIGHTNESS_CONTROL_PIN  = 26; // BUTTON_1 on schematic
    constexpr int BUTTON_2                = 27; // BUTTON_2 on schematic
    constexpr int BTN_PRESET_READ         = 28; // Pedals recal button
    constexpr int BTN_START_READ          = 29; // RTD on schematic
    constexpr int BTN_DATA_READ           = 30; // DATA_MARK on schematic
    constexpr int BTN_MC_CYCLE_READ       = 31; // DB/MC_RESET on schematic
    constexpr int BUZZER_CONTROL_PIN      = 32;

    /* Watchdog Interface */
    constexpr pin WATCHDOG_KICK_PIN = 36;
    constexpr pin SOFTWARE_OK_PIN = 37; // Watchdog's !MR pin
}

// calibration and processing constants
namespace VCFSystems
{
    /* IO Expander */
    constexpr uint8_t IO_EXPANDER_ADDR = 0x20;

    /* Neopixel Controller */
    constexpr int NEOPIXEL_CONTROL_PIN = 33;
    constexpr int NEOPIXEL_COUNT = 16; // 16 neopixeles on dashboard

    /* EEPROM addresses */
    constexpr uint32_t ACCEL_1_MIN_ADDR = 0;
    constexpr uint32_t ACCEL_2_MIN_ADDR = 4;
    constexpr uint32_t ACCEL_1_MAX_ADDR = 8;
    constexpr uint32_t ACCEL_2_MAX_ADDR = 12;

    constexpr uint32_t BRAKE_1_MIN_ADDR = 16;
    constexpr uint32_t BRAKE_2_MIN_ADDR = 20;
    constexpr uint32_t BRAKE_1_MAX_ADDR = 24;
    constexpr uint32_t BRAKE_2_MAX_ADDR = 28;

    constexpr uint32_t ACCEL_MIN_SENSOR_PEDAL_1 = 90;
    constexpr uint32_t ACCEL_MIN_SENSOR_PEDAL_2 = 90;
    constexpr uint32_t ACCEL_MAX_SENSOR_PEDAL_1 = 4000;
    constexpr uint32_t ACCEL_MAX_SENSOR_PEDAL_2 = 4000;

    constexpr uint32_t BRAKE_MIN_SENSOR_PEDAL_1 = 90;
    constexpr uint32_t BRAKE_MIN_SENSOR_PEDAL_2 = 90;
    constexpr uint32_t BRAKE_MAX_SENSOR_PEDAL_1 = 4000;
    constexpr uint32_t BRAKE_MAX_SENSOR_PEDAL_2 = 4000;


    // Steering System Constants
    /* Steering */
    constexpr float ANALOG_TOLERANCE = 0.05f;  // +/- 5% sensor tolerance
    constexpr float DIGITAL_TOLERANCE = 0.05f;  // +/- 0.2 degree error
    constexpr degree ERROR_BETWEEN_SENSORS_TOLERANCE = 5.0f;   // degrees
    constexpr degree MAX_DTHETA_THRESHOLD = 50.0f;  // max angle change per sample (degrees)
    constexpr degree DEG_PER_COUNT_DIGITAL = 360.0f / 16384.0f;
    constexpr degree DEG_PER_COUNT_ANALOG = 360.0f / 3686.4f;

    constexpr uint32_t MIN_STEERING_SIGNAL_ANALOG_ADDR = 56;
    constexpr uint32_t MAX_STEERING_SIGNAL_ANALOG_ADDR = 60;
    constexpr uint32_t MIN_STEERING_SIGNAL_DIGITAL_ADDR = 32;
    constexpr uint32_t MAX_STEERING_SIGNAL_DIGITAL_ADDR = 36;

    constexpr int32_t ANALOG_MIN_WITH_MARGINS_ADDR = 40;
    constexpr int32_t ANALOG_MAX_WITH_MARGINS_ADDR = 44;
    constexpr int32_t DIGITAL_MIN_WITH_MARGINS_ADDR = 48;
    constexpr int32_t DIGITAL_MAX_WITH_MARGINS_ADDR = 52;

    /* Pedals System */
    constexpr float ACCEL_ACTIVATION_PERCENTAGE            = 0.10f;
    constexpr float ACCEL_DEADZONE_MARGIN                  = 0.03f;
    constexpr float ACCEL_MECHANICAL_ACTIVATION_PERCENTAGE = 0.05f;
    constexpr float BRAKE_ACTIVATION_PERCENTAGE            = 0.50f;
    constexpr float BRAKE_DEADZONE_MARGIN                  = 0.04f;
    constexpr float BRAKE_MECHANICAL_ACTIVATION_PERCENTAGE = 0.5f;

    /* Load cells */
    constexpr float LBS_TO_NEWTONS = 4.4482216153f;

}

// software configuration constants
namespace VCFConstants
{
    /* Task Times */
    constexpr uint8_t WATCHDOG_PRIORITY = 1;
    constexpr time_us WATCHDOG_KICK_PERIOD_US = 10000; // 10 000 us = 100 Hz

    constexpr uint8_t ASYNC_MAIN_PRIORITY = 2;
    constexpr time_us ASYNC_MAIN_PERIOD_US = 100; // 100 us = 10 kHz

    constexpr uint8_t PEDALS_SAMPLE_PRIORITY = 3;
    constexpr time_us PEDALS_SAMPLE_PERIOD_US = 500; // 500 us = 2 kHz

    constexpr uint8_t STEERING_SAMPLE_PRIORITY = 4;
    constexpr time_us STEERING_SAMPLE_PERIOD_US = 1000; // 1 000 us = 1 kHz

    constexpr uint8_t LOADCELL_SAMPLE_PRIORITY = 5;
    constexpr time_us LOADCELL_SAMPLE_PERIOD_US = 250; // 250 us = 4 kHz

    constexpr uint8_t PEDALS_SEND_PRIORITY = 6;
    constexpr time_us PEDALS_SEND_PERIOD_US = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t STEERING_SEND_PRIORITY = 7;
    constexpr time_us STEERING_SEND_PERIOD_US = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t LOADCELL_SEND_PRIORITY = 8;
    constexpr time_us LOADCELL_SEND_PERIOD_US = 4000; // 4 000 = 250 Hz

    constexpr uint8_t CAN_SEND_PRIORITY = 9;
    constexpr time_us CAN_SEND_PERIOD_US = 2000; // 2 000 us = 500 Hz

    constexpr uint8_t DASH_SEND_PRIORITY = 10;
    constexpr time_us DASH_SEND_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t DASH_SAMPLE_PRIORITY = 11;
    constexpr time_us DASH_SAMPLE_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t ETHERNET_SEND_PRIORITY = 12;
    constexpr time_us ETHERNET_SEND_PERIOD_US = 100000; // 100 000 us = 10Hz

    constexpr uint8_t BUZZER_PRIORITY = 13;
    constexpr time_us BUZZER_WRITE_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t NEOPIXEL_UPDATE_PRIORITY = 14;
    constexpr time_us NEOPIXEL_UPDATE_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t PEDALS_RECALIBRATION_PRIORITY = 20;
    constexpr time_us PEDALS_RECALIBRATION_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t STEERING_RECALIBRATION_PRIORITY = 21;
    constexpr time_us STEERING_RECALIBRATION_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t DEBUG_PRIORITY = 50;
    constexpr time_us DEBUG_PERIOD_US = 10000; // 10 000 us = 2 Hz

    /* CAN Constants */
    constexpr uint32_t TELEM_CAN_BAUDRATE = 1000000; // 1 000 000 = 1 Mbit/s
    constexpr uint32_t FAUX_CAN_BAUDRATE = 500000; // 500 000 = 500 Kbit/s
}

#endif /* VCF_CONSTANTS */
