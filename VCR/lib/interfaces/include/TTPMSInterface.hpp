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
 * Izze-Racing HS-TTPMS-V2 sensors / W-REC-V2 receiver (datasheet Rev.D). Read-only: decodes sensor data.
 * Receiver programming (CAN base ID, bit rate) lives in the vcr_ttpms_test env (main_ttpms_test.cpp).
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

    constexpr uint16_t MIN_BASE_CAN_ID = 0x001;
    constexpr uint16_t MAX_DECODE_BASE_CAN_ID = 0x7FF - (NUM_TTPMS_IDS - 1); // all four sensor blocks must fit in 11 bits
}

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

struct TTPMSParams_s
{
    /* Health thresholds */
    uint16_t low_battery_threshold_mv;
    int16_t min_rssi_dbm;

    /* Conversions */
    float hs_pressure_scale_mbar;
    float hs_pressure_offset_mbar;
    float temp_scale_degc;
    float temp_offset_degc;

    /* Decode base ID limits */
    uint16_t min_base_can_id;
    uint16_t max_decode_base_can_id;
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
            .low_battery_threshold_mv = ttpms_default_params::LOW_BATTERY_THRESHOLD_MV,
            .min_rssi_dbm = ttpms_default_params::MIN_RSSI_DBM,
            .hs_pressure_scale_mbar = ttpms_default_params::HS_PRESSURE_SCALE_MBAR,
            .hs_pressure_offset_mbar = ttpms_default_params::HS_PRESSURE_OFFSET_MBAR,
            .temp_scale_degc = ttpms_default_params::TEMP_SCALE_DEGC,
            .temp_offset_degc = ttpms_default_params::TEMP_OFFSET_DEGC,
            .min_base_can_id = ttpms_default_params::MIN_BASE_CAN_ID,
            .max_decode_base_can_id = ttpms_default_params::MAX_DECODE_BASE_CAN_ID
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

private:

    TTPMSSingleSensorData_s &_sensorAt(size_t index);

    /**
     * @brief Checks if the battery health or RSSI (Received Signal Strength Indicator) of the sensors is too low.
     */
    void _updateHealth(TTPMSSingleSensorData_s &sensor) const;

    float _hsPressureFromRaw(uint16_t raw) const;
    float _tempFromRaw(uint16_t raw) const;

    TTPMSAllSensorData_s _ttpms_data;
    const TTPMSParams_s _params = {};
    uint16_t _base_can_id = ttpms_default_params::DEFAULT_BASE_CAN_ID;

};
using TTPMSInterfaceInstance = etl::singleton<TTPMSInterface>;

#endif // TTPMSINTERFACE_H
