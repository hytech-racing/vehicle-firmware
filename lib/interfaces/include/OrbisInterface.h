#ifndef ORBIS_INTERFACE_H
#define ORBIS_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* Library Includes */
#include <Arduino.h>
#include "SharedFirmwareTypes.h"

using degree = float;


namespace orbis_constants
{
    constexpr uint32_t DEFAULT_BAUD_RATE  = 115200;
    constexpr uint16_t SELF_CALIB_MAX_TIME_MS = 10000;
    constexpr uint8_t TIMEOUT_MS = 10;
    constexpr uint8_t FACTORY_RESET_DELAY_MS = 200;
    constexpr uint8_t SAVE_CONFIG_DELAY_MS = 200;

    constexpr uint8_t OFFSET_HIGH_BYTE_SHIFT = 8;
    constexpr uint8_t POSITION_DATA_HIGH_BYTE_SHIFT = 8;
    constexpr uint8_t POSITION_DATA_RIGHT_SHIFT = 2;

    constexpr degree ANGLE_WRAPAROUND_THRESHOLD = 180.0f;
    constexpr degree FULL_ROTATION_DEGREES = 360.0f;
    constexpr degree DEGREES_PER_REVOLUTION = 360.0f;
    constexpr float ENCODER_RESOLUTION = 16384.0f; // 14-bit resolution = 2^14 counts per revolution
}

/**
 * @brief General errors are included in the first byte of the detailed position request response. Detailed errors are from the fourth byte.
 */
namespace orbis_bitmasks
{
    const uint8_t GENERAL_WARNING_BITMASK = 0b00000001; // Error if low, position data is valid, but some operating conditions are close to limits
    const uint8_t GENERAL_ERROR_BITMASK   = 0b00000010; // Error if low, position data is not valid

    const uint8_t OFFSET_RECOMBINING_MASK = 0xFF;

    const uint8_t DETAILED_COUNTER_ERROR_BITMASK  = 0b00001000;  // Errors if high
    const uint8_t DETAILED_SPEED_HIGH_BITMASK     = 0b00010000;  // Errors if high
    const uint8_t DETAILED_TEMP_RANGE_BITMASK     = 0b00100000;  // Errors if high
    const uint8_t DETAILED_DIST_FAR_BITMASK       = 0b01000000;  // Errors if high
    const uint8_t DETAILED_DIST_NEAR_BITMASK      = 0b10000000;  // Errors if high

    const uint8_t SELF_CALIB_STATUS_BITMASK           = 0b00000011;
    const uint8_t SELF_CALIB_NEW_COUNTER_BITMASK      = 0b00000011;
    const uint8_t SELF_CALIB_TIMEOUT_ERROR_BITMASK    = 0b00000100;
    const uint8_t SELF_CALIB_PARAMETER_ERROR_BITMASK  = 0b00001000;
}

namespace orbis_commands
{
    const byte UNLOCK_SEQUENCE[4]            = {0xCD, 0xEF, 0x89, 0xAB};
    const byte SELF_CALIB_START              = 0x41; //      requires unlock sequence
    const byte SELF_CALIB_STATUS             = 0x69;
    const byte POSITION_OFFSET               = 0x5A; // 'Z'  requires unlock sequence
    const byte SAVE_CONFIG                   = 0x63; // 'c'  requires unlock sequence
    const byte FACTORY_RESET                 = 0x72; // 'r'  requires unlock sequence
    const byte SHORT_POS_REQUEST             = 0x33;
    const byte DETAILED_POS_REQUEST          = 0x64;
    const byte MULTITURN_COUNTER_SETTING     = 0x4D; // 'M'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_SETTING   = 0x54; // 'T'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_START     = 0x53; // 'S'  requires unlock sequence
    const byte CONTINUOUS_RESPONSE_STOP      = 0x50; // 'P'  requires unlock sequence
    const byte BAUD_RATE_SETTING             = 0x42; // 'B'  requires unlock sequence
}

struct OrbisErrorFlags_s
{
    bool calibration_timeout       = false;  // Ring did not make complete during 10 seconds
    bool calibration_parameter     = false;  // Mechanical installation outside tolerance
    bool counter_error             = false;  // Multiturn counter error (bit=1 means error)
    bool speed_high                = false;  // Speed too high (bit=1 means error)
    bool temp_out_of_range         = false;  // Temperature out of range (bit=1 means error)
    bool dist_far                  = false;  // Dist b/w readhead and ring too far (bit=1 means error)
    bool dist_near                 = false;  // Dist b/w readhead and ring too close (bit=1 means error)
};

class OrbisInterface
{
public:

    OrbisInterface(HardwareSerial* serial);

    OrbisErrorFlags_s getOrbisDetailedErrors() const { return _orbisErrors; }

    bool performSelfCalibration();

    void setEncoderOffset();

    void saveConfiguration();

    void factoryReset();

    void sample();

    SteeringEncoderReading_s getLastReading();

private:

    HardwareSerial* _serial;
    SteeringEncoderReading_s _lastReading; // Most recently sampled encoder reading.
    OrbisErrorFlags_s _orbisErrors;

    void _decodeErrors(uint8_t general, uint8_t detailed);
    void _sendUnlockSequence();
    void _flushSerialBuffer();

};

using OrbisInterfaceInstance = etl::singleton<OrbisInterface>;

#endif /* ORBIS_BR_H */