#ifndef TTPMSINTERFACE_H
#define TTPMSINTERFACE_H

/* Standard Library */
#include <stdint.h>
#include <array>

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <FlexCAN_T4.h>
#include "hytech.h"

#define TEMP_CHANNELS 16

/**
 * Constants from the Izze-Racing HS-TTPMS-V2 / W-REC-V2 datasheet (Rev. D).
 */
namespace TTPMSConstants
{
    /* Health thresholds (datasheet "Additional Information") */
    constexpr uint16_t LOW_BATTERY_THRESHOLD_MV = 2800; // Replace battery below ~2800 mV
    constexpr int16_t MIN_RSSI_DBM = -90;               // Packets drop below -90 dBm average RSSI
    constexpr float MAX_SENSOR_TEMP_C = 120.0f;         // Max recommended sensor temp for reliability/battery life

    /* HS pressure: 0.1 mBar per bit, +3000 mBar offset, signed */
    constexpr float HS_PRESSURE_SCALE_MBAR = 0.1f;
    constexpr float HS_PRESSURE_OFFSET_MBAR = 3000.0f;

    /* Receiver programming constants (datasheet Appendix A.1 - A.3) */
    constexpr uint16_t BASE_CONFIG_PROGRAMMING_CONSTANT = 30000;      // 0x7530
    constexpr uint16_t RF_CONFIG_PROGRAMMING_CONSTANT = 20020;        // 0x4E34
    constexpr uint16_t EMISSIVITY_T1_T6_PROGRAMMING_CONSTANT = 20030;  // 0x4E3E
    constexpr uint16_t EMISSIVITY_T7_T12_PROGRAMMING_CONSTANT = 20031; // 0x4E3F
    constexpr uint16_t EMISSIVITY_T13_T16_PROGRAMMING_CONSTANT = 20032; // 0x4E40
    constexpr uint8_t SENSOR_ASSIGNMENT_DEFAULT = 1;
    constexpr uint8_t DEFAULT_EMISSIVITY_RAW = 0x4C; // 0.76, datasheet says DO NOT CHANGE
    constexpr uint16_t MIN_BASE_CAN_ID = 0x001;
    constexpr uint16_t MAX_BASE_CAN_ID = 0x7FF - 5; // receiver uses base ID through base ID + 5
    constexpr uint32_t MIN_RF_FREQUENCY_HZ = 868000000;
    constexpr uint32_t MAX_RF_FREQUENCY_HZ = 930000000;
    constexpr uint32_t RF_FREQUENCY_RESOLUTION_HZ = 100000; // 1 bit = 10^5 Hz
    constexpr float MIN_EMISSIVITY = 0.01f;
    constexpr float MAX_EMISSIVITY = 1.0f;

    /* Configuration sequence: send at 1 Hz for at least 10 seconds */
    constexpr unsigned long CONFIG_SEND_PERIOD_MS = 1000;
    constexpr unsigned long CONFIG_SEND_DURATION_MS = 11000; // 1s margin over the 10s minimum
    constexpr size_t MAX_CONFIG_MSGS = 4;
}

enum class TTPMSCorner_e
{
    LF = 0,
    RF = 1,
    LR = 2,
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
    /* Message 1 */
    uint16_t serial_number;
    uint16_t bat_voltage;       // mV
    float pressure;             // High-speed absolute pressure, mBar
    float gauge_pressure;       // mBar
    /* Messages 2-5 */
    std::array<float, TEMP_CHANNELS> temp_data; // Infrared temps CH1 (inside) - CH16 (outside), degC
    /* Message 6 */
    uint16_t transmission_count;
    int16_t rssi;               // dBm
    float sensor_temp;          // Internal sensor temperature, degC
    uint16_t node_id;

    /* Derived health status */
    bool battery_low;           // bat_voltage below LOW_BATTERY_THRESHOLD_MV
    bool rssi_weak;             // rssi below MIN_RSSI_DBM
    bool sensor_over_temp;      // sensor_temp above MAX_SENSOR_TEMP_C

    /* Receive timestamps */
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

    TTPMSInterface() : _ttpms_data{} {};

    /**
     * Retrieves the latest data that has been sent from the sensors
     * @return the latest temp data
     */
    TTPMSAllSensorData_s getTTPMSData() const;

    /**
     * @brief CAN receive function to parse the new CAN msg and update internal state
     * @param msg the CAN msg to parse
     * @param curr_millis the current system time in ms
    */
    void receiveTTPMSData(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @brief Method to print all TTMPS data for a single wheel
     * @param wheel_label is a string (i.e. FL, FR, etc.)
     * @param sensor is a reference to data struct for a sensor
    */
    void printTTPMSData(const char* wheel_label, const TTPMSSingleSensorData_s &sensor);

    /**
     * @brief Returns true if a sensor has sent its pressure message within timeout_ms.
     * Note: sensors sleep when uninflated/cold and only send every 240s when inflated and idle,
     * so pick timeout_ms according to the expected operating state.
     */
    static bool isSensorActive(const TTPMSSingleSensorData_s &sensor, unsigned long curr_millis, unsigned long timeout_ms);

    /**
     * @brief Returns the current base CAN ID of the receiver for a given corner (default IDs)
     */
    static uint16_t getReceiverBaseID(TTPMSCorner_e corner);

    /**
     * @brief Builds the basic receiver programming message (datasheet A.1)
     * @param corner the receiver to program (message is sent on that receiver's current base ID)
     * @param new_base_id new 11-bit base CAN ID
     * @param bit_rate new CAN bit rate
     * @param emissivity_mode emissivity mode (must be CUSTOM for custom emissivity to take effect)
     * @param msg_out the packed CAN message
     * @return false if any argument is out of range
     */
    static bool buildBaseConfigMsg(TTPMSCorner_e corner, uint16_t new_base_id, TTPMSBitRate_e bit_rate,
                                   TTPMSEmissivityMode_e emissivity_mode, CAN_message_t &msg_out);

    /**
     * @brief Builds the network ID / node ID / RF frequency programming message (datasheet A.2)
     * @param frequency_hz RF frequency in Hz, 868 - 930 MHz in 100 kHz steps
     * @return false if any argument is out of range
     */
    static bool buildRFConfigMsg(TTPMSCorner_e corner, uint8_t network_id, uint8_t node_id,
                                 uint32_t frequency_hz, CAN_message_t &msg_out);

    /**
     * @brief Builds the three custom emissivity programming messages (datasheet A.3)
     * @param emissivity effective emissivity per IR channel, 0.01 - 1.00
     * @return false if any emissivity is out of range
     */
    static bool buildEmissivityConfigMsgs(TTPMSCorner_e corner, const std::array<float, TEMP_CHANNELS> &emissivity,
                                          std::array<CAN_message_t, 3> &msgs_out);

    /**
     * @brief Starts a receiver configuration sequence. The given messages are enqueued on the
     * TTPMS (front aux) CAN bus at 1 Hz for CONFIG_SEND_DURATION_MS by updateConfigSequence().
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

    static void _updateHealth(TTPMSSingleSensorData_s &sensor);

    static float _hsPressureFromRaw(uint16_t raw);

    static void _packProgrammingConstant(CAN_message_t &msg, TTPMSCorner_e corner, uint16_t programming_constant);

    TTPMSAllSensorData_s _ttpms_data;

    std::array<CAN_message_t, TTPMSConstants::MAX_CONFIG_MSGS> _config_msgs;
    size_t _num_config_msgs = 0;
    bool _config_active = false;
    unsigned long _config_start_millis = 0;
    unsigned long _last_config_send_millis = 0;

};

using TTPMSInterfaceInstance = etl::singleton<TTPMSInterface>;

#endif // __TTPMSINTERFACE_H__
