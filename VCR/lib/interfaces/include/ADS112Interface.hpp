#ifndef ADS112_INTERFACE
#define ADS112_INTERFACE

#include "AnalogSensorsInterface.h"
#include "etl/singleton.h"

#include <Arduino.h>
#include <array>
#include <cstdint>


namespace ads112_bit_shift_params
{
    constexpr uint8_t DATA_RATE_SHIFT = 5U;
    constexpr uint8_t NORMAL_MODE_SHIFT = 4U;
    constexpr uint8_t SINGLE_SHOT_SHIFT = 3U;
    constexpr uint8_t REFERENCE_SHIFT = 1U;
} // namespace ads112_bit_shift_params


namespace ads112_default_parameters
{
    constexpr int ADS112U04_NUM_CHANNELS = 4;
    constexpr int ADS112U04_PIN_UNUSED = -1;
    constexpr uint32_t ADS112U04_BAUDRATE = 115200;

    constexpr uint8_t ADS112U04_SYNC_WORD = 0x55;
    constexpr uint8_t ADS112U04_RESET_COMMAND = 0x06;
    constexpr uint8_t ADS112U04_START_SYNC_COMMAND = 0x08;
    constexpr uint8_t ADS112U04_POWERDOWN_COMMAND = 0x02;
    constexpr uint8_t ADS112U04_RDATA_COMMAND = 0x10;
    constexpr uint8_t ADS112U04_WREG_BASE_COMMAND = 0x40;
    constexpr uint8_t ADS112U04_RREG_BASE_COMMAND = 0x20;
    constexpr uint8_t ADS112U04_CONFIG_REG_4_DEFAULT = 0x48U;

    constexpr uint32_t ADS112U04_POWER_UP_DELAY_US = 600;
    constexpr uint32_t ADS112U04_RESET_DELAY_MS = 50;
    constexpr uint32_t ADS112U04_READ_TIMEOUT_MS = 100;

    // Conversion wait time in ms per data rate.
    // 20 SPS -> 70ms (50ms nominal + margin)
    // 330 SPS -> 10ms (3ms nominal + margin)
    constexpr uint32_t ADS112U04_CONVERSION_WAIT_MS = 70;

    // Gap between bytes sent over UART — gives the ADC time to detect
    // baud rate from the sync word and latch each command on the stop bit.
    constexpr uint32_t ADS112U04_BYTE_GAP_US = 25;

    constexpr float ADS112U04_REFERENCE_VOLTAGE = 5.0f;
    constexpr float ADS112U04_GAIN = 1.0f;
    constexpr float ADS112U04_FULL_SCALE_COUNTS = 32768.0f;

    constexpr uint16_t ADS112U04_TEMP_SIGN_BIT_MASK = 0x2000U;
    constexpr uint16_t ADS112U04_TEMP_SIGN_EXTEND_MASK = 0xC000U;
    constexpr float ADS112U04_TEMP_SCALE = 0.03125f;
} // namespace ads112_default_parameters


enum class ADS112U04InputMux_e : uint8_t
{
    AIN0_AVSS = 0x08,
    AIN1_AVSS = 0x09,
    AIN2_AVSS = 0x0A,
    AIN3_AVSS = 0x0B
};

enum class ADS112U04Gain_e : uint8_t
{
    GAIN_1 = 0x00,
    GAIN_2 = 0x01,
    GAIN_4 = 0x02,
    GAIN_8 = 0x03,
    GAIN_16 = 0x04,
    GAIN_32 = 0x05,
    GAIN_64 = 0x06,
    GAIN_128 = 0x07
};

enum class ADS112U04DataRate_e : uint8_t
{
    SPS_20 = 0x00,
    SPS_45 = 0x01,
    SPS_90 = 0x02,
    SPS_175 = 0x03,
    SPS_330 = 0x04,
    SPS_600 = 0x05,
    SPS_1000 = 0x06
};

enum class ADS112U04Reference_e : uint8_t
{
    INTERNAL_2V048 = 0x00,
    EXTERNAL_REFP_REFN = 0x01,
    ANALOG_SUPPLY = 0x02
};

struct ADS112Pinout_s
{
    int reset_pin;
    int drdy_pin;
};

struct ADS112Config_s
{
    std::array<ADS112U04InputMux_e, ads112_default_parameters::ADS112U04_NUM_CHANNELS> muxes;
    ADS112U04Gain_e gain;
    ADS112U04DataRate_e data_rate;
    ADS112U04Reference_e reference;
    float reference_voltage;
    float gain_value;
    float full_scale_counts;
    float internal_temp_scale;
    uint16_t temp_sign_bit_mask;
    uint16_t temp_sign_extend_mask;
    uint8_t config_reg_4_default_value;
};

struct ADS112Commands_s
{
    uint8_t sync_word;
    uint8_t reset_command;
    uint8_t start_sync_command;
    uint8_t powerdown_command;
    uint8_t rdata_command;
    uint8_t wreg_base_command;
    uint8_t rreg_base_command;
};

struct ADS112Timing_s
{
    uint32_t power_up_delay_us;
    uint32_t reset_delay_ms;
    uint32_t read_timeout_ms;
    uint32_t conversion_wait_ms;
    uint32_t byte_gap_us;
};

struct ADS112TemperatureReading_s
{
    int16_t raw_temperature_count = 0;
    float temperature_c = 0.0f;
    bool data_valid = false;
};

enum class ADS112State_e : uint8_t
{
    // Initialisation sequence
    INIT_RESET,      // send RESET command, start reset wait
    INIT_WAIT_RESET, // waiting for reset delay to expire
    INIT_WRITE_REG0, // write config register 0
    INIT_WRITE_REG1, // write config register 1
    INIT_WRITE_REG2, // write config register 2
    INIT_WRITE_REG3, // write config register 3
    INIT_WRITE_REG4, // write config register 4

    // Normal sampling sequence (repeated for each channel)
    IDLE,             // ready to start a new channel conversion
    START_CONVERSION, // send START command for current channel
    WAIT_CONVERSION,  // waiting for conversion time to elapse
    REQUEST_DATA,     // send RDATA command
    READ_DATA,        // waiting for 2 RX bytes
    PROCESS_DATA,     // assemble raw count, advance to next channel

    // Error / fault
    FAULT // communication error, stops ticking
};

class ADS112Interface : public AnalogMultiSensor<ads112_default_parameters::ADS112U04_NUM_CHANNELS>
{
  public:
    static constexpr int NUM_CHANNELS = ads112_default_parameters::ADS112U04_NUM_CHANNELS;

    ADS112Interface(
        HardwareSerial& serial,
        const std::array<float, NUM_CHANNELS>& scales,
        const std::array<float, NUM_CHANNELS>& offsets,
        ADS112Pinout_s pinouts = {ads112_default_parameters::ADS112U04_PIN_UNUSED, ads112_default_parameters::ADS112U04_PIN_UNUSED},
        ADS112Config_s config =
            {{ADS112U04InputMux_e::AIN0_AVSS, ADS112U04InputMux_e::AIN1_AVSS, ADS112U04InputMux_e::AIN2_AVSS, ADS112U04InputMux_e::AIN3_AVSS},
             ADS112U04Gain_e::GAIN_1,
             ADS112U04DataRate_e::SPS_20,
             ADS112U04Reference_e::ANALOG_SUPPLY,
             ads112_default_parameters::ADS112U04_REFERENCE_VOLTAGE,
             ads112_default_parameters::ADS112U04_GAIN,
             ads112_default_parameters::ADS112U04_FULL_SCALE_COUNTS,
             ads112_default_parameters::ADS112U04_TEMP_SCALE,
             ads112_default_parameters::ADS112U04_TEMP_SIGN_BIT_MASK,
             ads112_default_parameters::ADS112U04_TEMP_SIGN_EXTEND_MASK,
             ads112_default_parameters::ADS112U04_CONFIG_REG_4_DEFAULT},
        ADS112Commands_s commands =
            {ads112_default_parameters::ADS112U04_SYNC_WORD,
             ads112_default_parameters::ADS112U04_RESET_COMMAND,
             ads112_default_parameters::ADS112U04_START_SYNC_COMMAND,
             ads112_default_parameters::ADS112U04_POWERDOWN_COMMAND,
             ads112_default_parameters::ADS112U04_RDATA_COMMAND,
             ads112_default_parameters::ADS112U04_WREG_BASE_COMMAND,
             ads112_default_parameters::ADS112U04_RREG_BASE_COMMAND},
        ADS112Timing_s timing =
            {ads112_default_parameters::ADS112U04_POWER_UP_DELAY_US,
             ads112_default_parameters::ADS112U04_RESET_DELAY_MS,
             ads112_default_parameters::ADS112U04_READ_TIMEOUT_MS,
             ads112_default_parameters::ADS112U04_CONVERSION_WAIT_MS,
             ads112_default_parameters::ADS112U04_BYTE_GAP_US},
        uint32_t baud_rate = ads112_default_parameters::ADS112U04_BAUDRATE);

    // Call once at startup — starts the init state machine (non-blocking).
    void init();

    // Call every loop iteration. Drives the state machine forward.
    void tick() override;

    // Returns the most recent converted value for a channel.
    AnalogConversion_s get_channel(size_t channel);

    // True once the init sequence is complete and sampling is running.
    bool is_ready() const;

    // True if a communication fault has been detected.
    bool is_faulted() const;

    // Current state machine state (useful for debugging).
    ADS112State_e get_state() const;

    void power_down();

    ADS112TemperatureReading_s get_internal_temperature();

  private:
    HardwareSerial* _serial;
    ADS112Pinout_s _pinouts;
    ADS112Config_s _config;
    ADS112Commands_s _commands;
    ADS112Timing_s _timing;
    uint32_t _baud_rate;

    ADS112State_e _state = ADS112State_e::INIT_RESET;
    int _current_channel = 0;
    uint32_t _state_entered_ms = 0; // millis() when we entered current state
    uint32_t _timeout_start_ms = 0; // for RX timeout tracking

    ADS112TemperatureReading_s _internal_temperature = {};

    // --------------- state machine helpers ----------------------------------

    // Send one byte then flush. Inserts byte-gap delay after flush.
    void _send_byte(uint8_t b);

    // Send sync word + command byte.
    void _send_command(uint8_t command);

    // Send sync word + WREG command + data byte.
    void _write_register(uint8_t register_address, uint8_t value);

    // Drain all bytes currently in the RX buffer.
    void _clear_rx();

    // Transition to a new state and record the entry time.
    void _enter_state(ADS112State_e next);

    // Build register byte values.
    uint8_t _build_config_register_0(ADS112U04InputMux_e mux) const;
    uint8_t _build_config_register_1(bool temperature_sensor_enabled = false) const;

    bool _has_reset_pin() const;
    bool _has_drdy_pin() const;

    // Required by AnalogMultiSensor — not used (state machine drives sampling).
    void _sample() override {}
};

using ADS112InterfaceInstance = etl::singleton<ADS112Interface>;

#endif /* ADS112_INTERFACE */