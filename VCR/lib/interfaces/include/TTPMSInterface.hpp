#ifndef TTPMSINTERFACE_H
#define TTPMSINTERFACE_H

/* Standard Library */
#include <stdint.h>
#include <stddef.h>
#include <array>

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <FlexCAN_T4.h>
#include "ht_can.h"

/**
 * Izze-Racing HS-TTPMS-V2 sensors / W-REC-V2 receiver (datasheet Rev.D).
 *
 * Each sensor occupies a block of 6 consecutive CAN IDs starting at its base ID:
 *   base + 0 : serial number, battery voltage, HS pressure, gauge pressure
 *   base + 1 : IR temp CH1  - CH4
 *   base + 2 : IR temp CH5  - CH8
 *   base + 3 : IR temp CH9  - CH12
 *   base + 4 : IR temp CH13 - CH16
 *   base + 5 : transmission count, RSSI, sensor temperature, sensor node ID
 *
 * The four sensors are laid out back to back starting at a single runtime-configurable
 * base ID, in the order FL, FR, RL, RR:
 *   FL = base, FR = base + 6, RL = base + 12, RR = base + 18   (24 IDs total)
 * The default base (0x424) matches the receiver factory defaults and the HT_CAN DBC
 * (FL/FR/RL/RR_TTPMS_1..6 = 0x424 - 0x43B).
 */

namespace ttpms_default_params
{
    constexpr size_t TEMP_CHANNELS = 16;
    constexpr size_t MSGS_PER_SENSOR = 6;
    constexpr size_t NUM_SENSORS = 4;
    constexpr uint16_t NUM_TTPMS_IDS = MSGS_PER_SENSOR * NUM_SENSORS; // 24

    constexpr uint16_t DEFAULT_BASE_CAN_ID = FL_TTPMS_1_CANID; // 0x424

    /* Health thresholds (datasheet "Additional Information") */
    constexpr uint16_t LOW_BATTERY_THRESHOLD_MV = 2800; // Replace battery below ~2800 mV
    constexpr int16_t MIN_RSSI_DBM = -90;               // Packets drop below -90 dBm average RSSI

    /* Data conversions (datasheet "CAN Specifications") */
    constexpr float HS_PRESSURE_SCALE_MBAR = 0.1f;      // signed, 0.1 mBar per bit
    constexpr float HS_PRESSURE_OFFSET_MBAR = 3000.0f;  // +3000 mBar offset
    constexpr float TEMP_SCALE_DEGC = 0.1f;             // unsigned, 0.1 degC per bit
    constexpr float TEMP_OFFSET_DEGC = -100.0f;         // -100 degC offset
    constexpr float MBAR_TO_PSI = 0.0145037738f;        // pressures are stored in psi

    /* Receiver command IDs, called "Programming Constant" in the datasheet (Appendix A.1 - A.3) */
    constexpr uint16_t BASE_CONFIG_COMMAND_ID = 0x7530;   // A.1: CAN Base ID, Sensor Assignment, Bit Rate, Emissivity mode
    constexpr uint16_t RF_CONFIG_COMMAND_ID = 0x4E34;     // A.2: Network, Node ID, and RF Frequency

    // "Emissivity" mode must be set to "Custom" in A.1 for these to take effect
    constexpr uint16_t EMISSIVITY_T1_T6_COMMAND_ID = 0x4E3E;
    constexpr uint16_t EMISSIVITY_T7_T12_COMMAND_ID = 0x4E3F;
    constexpr uint16_t EMISSIVITY_T13_T16_COMMAND_ID = 0x4E40;

    constexpr uint8_t SENSOR_ASSIGNMENT_DEFAULT = 1; // Only documented value
    constexpr uint8_t EMISSIVITY_RAW_DEFAULT = 0x4C; // 0.76, datasheet says DO NOT CHANGE

    constexpr uint16_t MIN_BASE_CAN_ID = 0x001;
    constexpr uint16_t MAX_RECEIVER_BASE_CAN_ID = 0x7FF - (MSGS_PER_SENSOR - 1);  // one sensor block must fit in 11 bits
    constexpr uint16_t MAX_DECODE_BASE_CAN_ID = 0x7FF - (NUM_TTPMS_IDS - 1);      // all four sensor blocks must fit in 11 bits

    constexpr uint32_t MIN_RF_FREQUENCY_HZ = 868'000'000;
    constexpr uint32_t MAX_RF_FREQUENCY_HZ = 930'000'000;
    constexpr uint32_t RF_FREQUENCY_RESOLUTION_HZ = 100'000; // 1 bit = 10^5 Hz

    constexpr float MIN_EMISSIVITY = 0.01f;  // 0x01
    constexpr float MAX_EMISSIVITY = 1.0f;   // 0x64

    /* Configuration sequence: send at 1 Hz for at least 10 seconds, then power cycle the receiver for 5 s */
    constexpr unsigned long CONFIG_SEND_PERIOD_MS = 1000;
    constexpr unsigned long CONFIG_SEND_DURATION_MS = 11000; // 1 s margin over the 10 s minimum
    constexpr size_t MAX_CONFIG_MSGS = 4;
}

/**
 * Command IDs sent in bytes 0-1 of receiver configuration messages ("Programming Constant" in datasheet Appendix A.1 - A.3)
 */
struct TTPMSCommandIDs_s
{
    uint16_t base_config;
    uint16_t rf_config;
    uint16_t emissivity_t1_t6;
    uint16_t emissivity_t7_t12;
    uint16_t emissivity_t13_t16;
};

struct TTPMSParams_s
{
    TTPMSCommandIDs_s command_ids;

    /* Health thresholds */
    uint16_t low_battery_threshold_mv;
    int16_t min_rssi_dbm;

    /* Conversions */
    float hs_pressure_scale_mbar;
    float hs_pressure_offset_mbar;
    float temp_scale_degc;
    float temp_offset_degc;

    /* Receiver programming */
    uint8_t sensor_assignment_default;
    uint8_t emissivity_raw_default;
    uint16_t min_base_can_id;
    uint16_t max_receiver_base_can_id;
    uint16_t max_decode_base_can_id;
    uint32_t min_rf_frequency_hz;
    uint32_t max_rf_frequency_hz;
    uint32_t rf_frequency_resolution_hz;
    float min_emissivity;
    float max_emissivity;

    /* Configuration sequence */
    unsigned long config_send_period_ms;
    unsigned long config_send_duration_ms;
};

/**
 * Sensor position. The value is the sensor's index in the consecutive CAN ID layout
 * (sensor base ID = decode base ID + index * MSGS_PER_SENSOR).
 */
enum class TTPMSCorner_e : uint8_t
{
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

enum class TTPMSBitRate_e : uint8_t
{
    BITRATE_1MBPS = 1,
    BITRATE_500KBPS = 2,
    BITRATE_250KBPS = 3,
    BITRATE_100KBPS = 4
};

enum class TTPMSEmissivityMode_e : uint8_t
{
    STANDARD = 0,
    DEFAULT_CAL = 1,
    CUSTOM = 2
};

struct TTPMSSingleSensorData_s
{
    /* Message 1 (base + 0) */
    uint16_t serial_number;
    uint16_t battery_voltage_mv;    // mV
    float pressure_psi;             // High-speed (absolute) pressure, psi
    float gauge_pressure_psi;       // psi
    /* Messages 2-5 (base + 1 .. base + 4) */
    std::array<float, ttpms_default_params::TEMP_CHANNELS> temp_data; // Infrared temps CH1 (inside) - CH16 (outside), degC
    /* Message 6 (base + 5) */
    uint16_t transmission_count;
    int16_t rssi_dbm;               // dBm
    float sensor_temp;              // Internal sensor temperature, degC
    uint16_t node_id;

    /* Derived health status */
    bool is_battery_low;           // battery below low_battery_threshold_mv
    bool is_rssi_weak;             // rssi below min_rssi_dbm

    /* Receive timestamps (0 = never received) */
    unsigned long last_pressure_recv_millis;    // Message 1
    unsigned long last_temp_recv_millis;        // Messages 2-5
    unsigned long last_status_recv_millis;      // Message 6
};

struct TTPMSAllSensorData_s
{
    TTPMSSingleSensorData_s fl_ttpms;
    TTPMSSingleSensorData_s fr_ttpms;
    TTPMSSingleSensorData_s rl_ttpms;
    TTPMSSingleSensorData_s rr_ttpms;
};

class TTPMSInterface
{
public:

    explicit TTPMSInterface(uint16_t base_can_id = ttpms_default_params::DEFAULT_BASE_CAN_ID)
      : _ttpms_data{},
        _params {
            .command_ids = {
                ttpms_default_params::BASE_CONFIG_COMMAND_ID,
                ttpms_default_params::RF_CONFIG_COMMAND_ID,
                ttpms_default_params::EMISSIVITY_T1_T6_COMMAND_ID,
                ttpms_default_params::EMISSIVITY_T7_T12_COMMAND_ID,
                ttpms_default_params::EMISSIVITY_T13_T16_COMMAND_ID
            },
            .low_battery_threshold_mv = ttpms_default_params::LOW_BATTERY_THRESHOLD_MV,
            .min_rssi_dbm = ttpms_default_params::MIN_RSSI_DBM,
            .hs_pressure_scale_mbar = ttpms_default_params::HS_PRESSURE_SCALE_MBAR,
            .hs_pressure_offset_mbar = ttpms_default_params::HS_PRESSURE_OFFSET_MBAR,
            .temp_scale_degc = ttpms_default_params::TEMP_SCALE_DEGC,
            .temp_offset_degc = ttpms_default_params::TEMP_OFFSET_DEGC,
            .sensor_assignment_default = ttpms_default_params::SENSOR_ASSIGNMENT_DEFAULT,
            .emissivity_raw_default = ttpms_default_params::EMISSIVITY_RAW_DEFAULT,
            .min_base_can_id = ttpms_default_params::MIN_BASE_CAN_ID,
            .max_receiver_base_can_id = ttpms_default_params::MAX_RECEIVER_BASE_CAN_ID,
            .max_decode_base_can_id = ttpms_default_params::MAX_DECODE_BASE_CAN_ID,
            .min_rf_frequency_hz = ttpms_default_params::MIN_RF_FREQUENCY_HZ,
            .max_rf_frequency_hz = ttpms_default_params::MAX_RF_FREQUENCY_HZ,
            .rf_frequency_resolution_hz = ttpms_default_params::RF_FREQUENCY_RESOLUTION_HZ,
            .min_emissivity = ttpms_default_params::MIN_EMISSIVITY,
            .max_emissivity = ttpms_default_params::MAX_EMISSIVITY,
            .config_send_period_ms = ttpms_default_params::CONFIG_SEND_PERIOD_MS,
            .config_send_duration_ms = ttpms_default_params::CONFIG_SEND_DURATION_MS
        }
    {
        if (!setBaseCANID(base_can_id))
        {
            _base_can_id = ttpms_default_params::DEFAULT_BASE_CAN_ID;
        }
    };

    /**
     * @brief Retrieves the latest data that has been sent from the sensors
     */
    TTPMSAllSensorData_s getTTPMSData() const;

    /**
     * @brief Sets the base CAN ID used for decoding. FL = base, FR = base + 6, RL = base + 12, RR = base + 18.
     * Clears all stored sensor data since it belonged to the old ID layout.
     * @return false if the 24-ID block would not fit in an 11-bit ID
     */
    bool setBaseCANID(uint16_t base_can_id);

    uint16_t getBaseCANID() const { return _base_can_id; }

    /**
     * @return the first CAN ID of a given sensor under the current decode base
     */
    uint16_t getSensorBaseCANID(TTPMSCorner_e corner) const;

    /**
     * @return true if the CAN ID falls in the current 24-ID TTPMS block
     */
    bool isTTPMSMessage(uint32_t can_id) const;

    /**
     * @brief CAN receive function to parse a TTPMS CAN msg and update internal state
     * @return true if the message was a TTPMS message and was consumed
    */
    bool receiveTTPMSData(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @return true if a sensor has sent its pressure message within timeout_ms.
     * @note Sensors sleep when uninflated/cold and only send every 240s when inflated and idle
     */
    bool isSensorActive(const TTPMSSingleSensorData_s &sensor, unsigned long curr_millis, unsigned long timeout_ms) const;

    /**
     * @brief Builds the basic receiver programming message (datasheet A.1)
     * @param target_can_id the CAN ID the receiver listens on for programming. The datasheet lists this as the
     *        receiver's current base ID (A.1: default 0x424; p.4 says 0x406), so it is passed explicitly.
     * @param new_base_id new 11-bit base CAN ID
     * @param bit_rate new CAN bit rate
     * @param emissivity_mode emissivity mode (must be CUSTOM for custom emissivity to take effect)
     * @param msg_out the packed CAN message
     * @return false if any argument is out of range
     */
    bool buildBaseConfigMsg(uint16_t target_can_id, uint16_t new_base_id, TTPMSBitRate_e bit_rate,
                            TTPMSEmissivityMode_e emissivity_mode, CAN_message_t &msg_out) const;

    /**
     * @brief Builds the network ID / node ID / RF frequency programming message (datasheet A.2)
     * @param frequency_hz RF frequency in Hz, 868 - 930 MHz in 100 kHz steps
     * @return false if any argument is out of range
     */
    bool buildRFConfigMsg(uint16_t target_can_id, uint8_t network_id, uint8_t node_id,
                          uint32_t frequency_hz, CAN_message_t &msg_out) const;

    /**
     * @brief Builds the three custom emissivity programming messages (datasheet A.3)
     * @param emissivity effective emissivity per IR channel, 0.01 - 1.00
     * @return false if any emissivity is out of range
     */
    bool buildEmissivityConfigMsgs(uint16_t target_can_id,
                                   const std::array<float, ttpms_default_params::TEMP_CHANNELS> &emissivity,
                                   std::array<CAN_message_t, 3> &msgs_out) const;

    /**
     * @brief Starts a receiver configuration sequence. The given messages are enqueued on the
     * rear aux CAN TX buffer at 1 Hz for config_send_duration_ms by updateConfigSequence().
     * After the sequence ends the receiver must be power cycled (off for 5s) to apply the change.
     * @return false if a sequence is already active or num_msgs is invalid
     */
    bool startConfigSequence(const CAN_message_t *msgs, size_t num_msgs, unsigned long curr_millis);

    /**
     * @brief Enqueues configuration messages while a configuration sequence is active. Safe to call periodically.
     */
    void updateConfigSequence(unsigned long curr_millis);

    bool isConfigSequenceActive() const { return _config_active; }

private:

    TTPMSSingleSensorData_s &_sensorAt(size_t index);

    /**
     * @brief Checks if the battery health or RSSI (Received Signal Strength Indicator) of the sensors is too low.
     */
    void _updateHealth(TTPMSSingleSensorData_s &sensor) const;

    float _hsPressureFromRaw(uint16_t raw) const;
    float _tempFromRaw(uint16_t raw) const;

    /**
     * @brief Clears a CAN message and packs the target ID and command ID (bytes 0-1)
    */
    void _packCommandID(CAN_message_t &msg, uint16_t target_can_id, uint16_t command_id) const;

    TTPMSAllSensorData_s _ttpms_data;
    const TTPMSParams_s _params = {};
    uint16_t _base_can_id = ttpms_default_params::DEFAULT_BASE_CAN_ID;

    std::array<CAN_message_t, ttpms_default_params::MAX_CONFIG_MSGS> _config_msgs;
    size_t _num_config_msgs = 0;
    bool _config_active = false;
    unsigned long _config_start_millis = 0;
    unsigned long _last_config_send_millis = 0;

};
using TTPMSInterfaceInstance = etl::singleton<TTPMSInterface>;

#endif // TTPMSINTERFACE_H
