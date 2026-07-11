#ifndef VCR_CONSTANTS
#define VCR_CONSTANTS

/* External Includes */
#include "SharedFirmwareTypes.h"

using pin = uint8_t;
using time_us = uint32_t;


namespace VCRInterfaces
{
    constexpr int ANALOG_RESOLUTION = 12;

    /* Misc. Pin Definitions */
    constexpr int INVERTER_ENABLE_PIN = 5;
    constexpr int FLOWMETER_PIN = 7;
    constexpr int BRAKELIGHT_CONTROL_PIN = 9;
    constexpr int MOTOR_COOLING_CONTROL_PIN = 33;
    constexpr int INVERTER_COOLING_CONTROL_PIN = 37;
    constexpr int BRAKE_HIGH_SENSE_PIN = 38;
    constexpr int CURRENT_HIGH_SENSE_PIN = 39;


    /* ---------- ADC Interface ---------- */
    constexpr unsigned int CHANNELS_WITHIN_MCP_ADC = 8;

    // Chip Selects
    constexpr int ADC0_CS = 10; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
    constexpr int ADC1_CS = 36; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.

    // Channels on ADC_0
    constexpr int RL_LOADCELL_CHANNEL     = 0;
    constexpr int RR_LOADCELL_CHANNEL     = 1;
    constexpr int RL_SUS_POT_CHANNEL      = 2;
    constexpr int RR_SUS_POT_CHANNEL      = 3;
    constexpr int GLV_SENSE_CHANNEL       = 4;
    constexpr int CURRENT_SENSE_CHANNEL   = 5;
    constexpr int REFERENCE_SENSE_CHANNEL = 6;
    // const int UNUSED_CHANNEL       = 7;

    // Channels on ADC_1
    constexpr int THERMISTOR_0_CHANNEL = 0;
    constexpr int THERMISTOR_1_CHANNEL = 1;
    constexpr int THERMISTOR_2_CHANNEL = 2;
    constexpr int THERMISTOR_3_CHANNEL = 3;
    constexpr int THERMISTOR_4_CHANNEL = 4;
    constexpr int THERMISTOR_5_CHANNEL = 5;
    constexpr int THERMISTOR_6_CHANNEL = 6;
    constexpr int THERMISTOR_7_CHANNEL = 7;

    // Scaling and Offsets
    constexpr float GLV_SENSE_SCALE = (float)(24.0/((2.77149877/3.3)*4096.0)); // unsure about the multiplication by 4.0865
    constexpr int GLV_SENSE_OFFSET = 0; // No offset for GLV
    constexpr float CURRENT_SENSE_SCALE = (float)(24/((2.77149877/3.3)*4096)); // unsure about the multiplication by 4.0865
    constexpr int CURRENT_SENSE_OFFSET = 0; // No offset for CURRENT_SENSE
    constexpr float REFERENCE_SENSE_SCALE = (float)(24/((2.77149877/3.3)*4096)); // unsure about the multiplication by 4.0865
    constexpr int REFERENCE_SENSE_OFFSET = 0; // No offset for REFERENCE_SENSE

    constexpr float LOADCELL_IIR_FILTER_ALPHA = 0.01f;
    constexpr float LBS_TO_NEWTONS = 4.4482216153;
    constexpr float RL_LOADCELL_SCALE = 1.0;
    constexpr float RL_LOADCELL_OFFSET = 0.0;
    constexpr float RR_LOADCELL_SCALE = 1.0;
    constexpr float RR_LOADCELL_OFFSET = 0;

    // Calibrated values measured from bolt to bolt
    constexpr float RL_SUS_POT_SCALE = 0.01459;
    constexpr float RL_SUS_POT_OFFSET = 148.5;
    constexpr float RR_SUS_POT_SCALE = 0.01337;
    constexpr float RR_SUS_POT_OFFSET = 152.5;

    // TODO: Figure out values
    const float THERMISTOR_0_SCALE = 1;
    const float THERMISTOR_0_OFFSET = 0;
    const float THERMISTOR_1_SCALE = 1;
    const float THERMISTOR_1_OFFSET = 0;
    const float THERMISTOR_2_SCALE = 1;
    const float THERMISTOR_2_OFFSET = 0;
    const float THERMISTOR_3_SCALE = 1;
    const float THERMISTOR_3_OFFSET = 0;
    const float THERMISTOR_4_SCALE = 1;
    const float THERMISTOR_4_OFFSET = 0;
    const float THERMISTOR_5_SCALE = 1;
    const float THERMISTOR_5_OFFSET = 0;
    const float THERMISTOR_6_SCALE = 1;
    const float THERMISTOR_6_OFFSET = 0;
    const float THERMISTOR_7_SCALE = 1;
    const float THERMISTOR_7_OFFSET = 0;

    // Coolant Temperature Sensor scale / offsets
    constexpr const float COOLANT_TEMP_SCALE = -31.3;
    constexpr const float COOLANT_TEMP_OFFSET = 242;
    constexpr const float TEST_TEMP_SCALE = -46.8;
    constexpr const float TEST_TEMP_OFFSET = 386;


    /* ---------- IOExpander Interface ---------- */
    constexpr uint8_t IOEXPANDER_I2C_ADDRESS = 0x20;

    // Port A modes
    constexpr uint8_t PORTA_DIRECTIONS = 0b11111111; // All inputs
    constexpr uint8_t PORTA_PULLUPS = 0b11111111;    // All pullups
    constexpr uint8_t PORTA_INVERTED = 0b00000000;   // None inverted

    // Port B modes
    constexpr uint8_t PORTB_DIRECTIONS = 0b11111111; // All inputs
    constexpr uint8_t PORTB_PULLUPS = 0b11111111;    // All pullups
    constexpr uint8_t PORTB_INVERTED = 0b00000000;   // None inverted


    /* ---------- Watchdog Interface ---------- */
    constexpr int WATCHDOG_PIN = 26;
    constexpr int SOFTWARE_OK_PIN = 27; // Watchdog's !RESET pin
    // NOTE: Consider moving hearbeat's into their respective interfaces
    constexpr unsigned long VCF_PEDALS_MAX_HEARTBEAT_MS = 50UL;  // 20ms at 60mph is about 0.5 meters
    constexpr uint64_t ACU_ACU_OK_MAX_HEARTBEAT_MS = 500;
}

namespace VCRSystems
{
    constexpr int INVERTER_MINIMUM_HV_VOLTAGE = 60;
    constexpr unsigned long MAX_ALLOWED_DB_LATENCY_MS = 40; // milliseconds
}

namespace VCRConstants
{
    /* ---------- Task Priorities & Periods ---------- */

    constexpr uint8_t WATCHDOG_PRIORITY = 1;
    constexpr time_us WATCHDOG_KICK_PERIOD_US = 10000; // 10 000 us = 100 Hz

    constexpr uint8_t ASYNC_MAIN_PRIORITY = 2;
    constexpr time_us ASYNC_MAIN_PERIOD_US = 100; // 100 us = 10 kHz

    constexpr uint8_t SEND_CAN_PRIORITY = 3;
    constexpr time_us SEND_CAN_PERIOD_US  = 1000; // 1 000 us = 1 000 Hz

    constexpr uint8_t AMS_PRIORITY = 4;
    constexpr time_us AMS_UPDATE_PERIOD_US = 5000; // 5 000 us = 200 Hz

    constexpr uint8_t BUZZER_PRIORITY = 5;
    constexpr time_us UPDATE_BUZZER_CONTROLLER_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t SUSPENSION_PRIORITY = 6;
    constexpr time_us SUSPENSION_CAN_PERIOD_US = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t INVERTER_SEND_PRIORITY = 7;
    constexpr time_us INVERTER_SEND_PERIOD_US = 5000; // 5 000 us = 200 Hz

    constexpr time_us ETHERNET_SEND_PRIORITY = 8;
    constexpr time_us ETHERNET_SEND_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr time_us ADC0_PRIORITY = 9;
    constexpr time_us ADC0_SAMPLE_PERIOD_US = 250; // 250 us = 4 kHz

    constexpr time_us CONTROLS_PRIORITY = 10;
    constexpr time_us CONTROLS_CAN_PERIOD_US = 4000; // 4 000 us = 250 Hz

    constexpr uint8_t COOLANT_TEMP_SEND_PRIORITY = 11;
    constexpr time_us COOLANT_TEMP_SEND_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t DASHBOARD_SEND_PRIORITY = 12;
    constexpr time_us DASHBOARD_SEND_PERIOD_US = 200000; // 200 000 us = 5 Hz

    constexpr uint8_t UPDATE_BRAKELIGHT_PRIORITY = 13;
    constexpr time_us UPDATE_BRAKELIGHT_PERIOD_US = 50000; // 50 000 us = 20 Hz

    constexpr uint8_t ADC1_PRIORITY = 14;
    constexpr time_us ADC1_SAMPLE_PERIOD_US = 10000;  // 10 000 us = 100 Hz

    constexpr uint8_t IOEXPANDER_PRIORITY = 15;
    constexpr time_us IOEXPANDER_SAMPLE_PERIOD_US = 5000; // 5 000 us = 200 Hz

    constexpr uint8_t DEBUG_PRIORITY = 50;
    constexpr time_us DEBUG_PERIOD_US = 10000; // 10 000 us = 2 Hz


    /* --------- CAN Constants ---------- */
    constexpr uint32_t TELEM_CAN_BAUDRATE = 1000000;
    constexpr uint32_t RAUX_CAN_BAUDRATE = 500000;
    constexpr uint32_t INVERTER_CAN_BAUDRATE = 500000;
}
#endif /* VCR_CONSTANTS */
