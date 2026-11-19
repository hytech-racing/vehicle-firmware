#ifndef IR_TEMP_INTERFACE_HPP
#define IR_TEMP_INTERFACE_HPP

/* Standard Library */
#include <algorithm>
#include <array>
#include <numeric>
#include <stddef.h>
#include <stdint.h>

/* External Includes */
#include "ht_can.h"

#include <FlexCAN_T4.h>

/**
 * Izze-Racing IRTS-V3 infrared temperature sensors (datasheet Rev.F).
 * Read-only: decodes sensor data. Sensor programming (CAN base ID, bit rate, ...) lives in the
 * vcf_ir_temp_test / vcr_ir_temp_test envs (main_ir_temp_test.cpp).
 *
 * Each sensor occupies a block of 4 consecutive CAN IDs starting at its base ID
 *   base + 0 : IR temp CH1  - CH4
 *   base + 1 : IR temp CH5  - CH8
 *   base + 2 : IR temp CH9  - CH12
 *   base + 3 : IR temp CH13 - CH16
 * Each channel is 16-bit big-endian, unsigned, 0.1 degC per bit, -100 degC offset.
 *
 * The four sensors are laid out back to back (0x4C4 - 0x4D3), in the order FL, FR, RL, RR.
 *
 * The RR sensor is the brake version (IRTS-60-V3 brake, datasheet Rev.D). It also sends
 *   base + 4 : sensor (package) temperature, bytes 0-1, same conversion. Keep below 85 degC.
 * Its base + 4 (0x4D4) lands past the end of the block, so it doesn't collide with another sensor.
 */

namespace ir_temp_default_params
{
    constexpr size_t TEMP_CHANNELS = 16;
    constexpr size_t CHANNELS_PER_MSG = 4;
    constexpr size_t MSGS_PER_SENSOR = TEMP_CHANNELS / CHANNELS_PER_MSG; // 4
    constexpr size_t NUM_SENSORS = 2;                                    // per board
    constexpr uint16_t NUM_IR_TEMP_IDS = MSGS_PER_SENSOR * NUM_SENSORS;  // 8

    /* Base CAN IDs (from the HT_CAN DBC; the sensors must be programmed to match) */
    constexpr uint16_t FL_BASE_CAN_ID = FL_IR_TEMP_CH1_CH4_CANID; // 0x4C4
    constexpr uint16_t FR_BASE_CAN_ID = FR_IR_TEMP_CH1_CH4_CANID; // 0x4C8
    constexpr uint16_t RL_BASE_CAN_ID = RL_IR_TEMP_CH1_CH4_CANID; // 0x4CC
    constexpr uint16_t RR_BASE_CAN_ID = RR_IR_TEMP_CH1_CH4_CANID; // 0x4D0

    /* Data conversions (datasheet "CAN Specifications") */
    constexpr float TEMP_SCALE_DEGC = 0.1f;     // unsigned, 0.1 degC per bit
    constexpr float TEMP_OFFSET_DEGC = -100.0f; // -100 degC offset
} // namespace ir_temp_default_params

/**
 * Sensor position on a board. The value is the sensor's index in the consecutive CAN ID layout
 * (sensor base ID = board base ID + index * MSGS_PER_SENSOR).
 */
enum class IRTempSensor_e : uint8_t
{
    LEFT = 0,  // VCF: FL, VCR: RL
    RIGHT = 1, // VCF: FR, VCR: RR
};

struct IRTempSingleSensorData_s
{
    std::array<float, ir_temp_default_params::TEMP_CHANNELS> temp_data; // CH1 - CH16, degC
    float max_temp;                                                     // degC
    float avg_temp;                                                     // degC
    unsigned long last_recv_millis;                                     // 0 = never received
    float brake_internal_sensor_temp;                                   // degC, package temperature (brake sensor only)
    unsigned long brake_internal_sensor_temp_last_recv_millis;          // 0 = never received
};

struct IRTempAllSensorData_s
{
    IRTempSingleSensorData_s left_sensor;  // VCF: FL, VCR: RL
    IRTempSingleSensorData_s right_sensor; // VCF: FR, VCR: RR
};

/**
 * @brief Shared decoder for a board's pair of IRTS-V3 sensors. Use the board-specific concrete classes.
 */
class IRTempInterface
{
  public:
    /**
     * @brief Retrieves the latest data that has been sent from the sensors
     */
    IRTempAllSensorData_s getIRTempData() const { return _ir_temp_data; }

    /**
     * @return the first CAN ID of a given sensor
     */
    uint16_t getSensorBaseCANID(IRTempSensor_e sensor) const
    {
        return static_cast<uint16_t>(_base_can_id + (static_cast<uint16_t>(sensor) * ir_temp_default_params::MSGS_PER_SENSOR));
    }

    /**
     * @return true if the CAN ID falls in this board's 8-ID IR temp block
     */
    bool isIRTempMessage(uint32_t can_id) const { return (can_id >= _base_can_id) && (can_id < (_base_can_id + ir_temp_default_params::NUM_IR_TEMP_IDS)); }

    /**
     * @brief CAN receive function to parse an IR temp CAN msg and update internal state
     * @return true if the message was an IR temp message and was consumed
     */
    bool receiveIRTempData(const CAN_message_t& can_msg, unsigned long curr_millis)
    {
        if (can_msg.flags.extended || !isIRTempMessage(can_msg.id) || can_msg.len < 8)
        {
            return false;
        }

        const uint32_t offset = can_msg.id - _base_can_id;
        const size_t sensor_index = offset / ir_temp_default_params::MSGS_PER_SENSOR;
        const size_t msg_index = offset % ir_temp_default_params::MSGS_PER_SENSOR;
        IRTempSingleSensorData_s& sensor = (sensor_index == 0) ? _ir_temp_data.left_sensor : _ir_temp_data.right_sensor;
        const uint8_t* buf = can_msg.buf; // NOLINT array decay to pointer. Values are 16-bit big-endian (high byte first)

        const size_t first_channel = msg_index * ir_temp_default_params::CHANNELS_PER_MSG;
        for (size_t i = 0; i < ir_temp_default_params::CHANNELS_PER_MSG; i++)
        {
            sensor.temp_data[first_channel + i] = _tempFromRaw((buf[i * 2] << 8) | buf[i * 2 + 1]);
        }

        sensor.max_temp = *std::max_element(sensor.temp_data.begin(), sensor.temp_data.end());
        sensor.avg_temp = std::accumulate(sensor.temp_data.begin(), sensor.temp_data.end(), 0.0f) / static_cast<float>(ir_temp_default_params::TEMP_CHANNELS);
        sensor.last_recv_millis = curr_millis;
        return true;
    }

    /**
     * @brief CAN receive function for a brake sensor's package temperature message (base + 4)
     */
    void receiveBrakeSensorTempData(const CAN_message_t& can_msg, IRTempSensor_e sensor_position, unsigned long curr_millis)
    {
        if (can_msg.flags.extended || can_msg.len < 2)
        {
            return;
        }

        IRTempSingleSensorData_s& sensor = (sensor_position == IRTempSensor_e::LEFT) ? _ir_temp_data.left_sensor : _ir_temp_data.right_sensor;
        const uint8_t* buf = can_msg.buf; // NOLINT array decay to pointer. 16-bit big-endian (high byte first)
        sensor.brake_internal_sensor_temp = _tempFromRaw((buf[0] << 8) | buf[1]);
        sensor.brake_internal_sensor_temp_last_recv_millis = curr_millis;
    }

    /**
     * @return true if a sensor has sent data within timeout_ms
     */
    bool isSensorActive(const IRTempSingleSensorData_s& sensor, unsigned long curr_millis, unsigned long timeout_ms) const
    {
        return (sensor.last_recv_millis != 0) && ((curr_millis - sensor.last_recv_millis) <= timeout_ms);
    }

  protected:
    /**
     * @param base_can_id base CAN ID of the board's left sensor. The right sensor follows at base_can_id + 4
     */
    explicit IRTempInterface(uint16_t base_can_id)
        : _ir_temp_data{}, _base_can_id(base_can_id)
    {
    }

  private:
    IRTempAllSensorData_s _ir_temp_data;
    const uint16_t _base_can_id;

    static float _tempFromRaw(uint16_t raw)
    {
        return (static_cast<float>(raw) * ir_temp_default_params::TEMP_SCALE_DEGC) + ir_temp_default_params::TEMP_OFFSET_DEGC;
    }
};

#endif // IR_TEMP_INTERFACE_HPP
