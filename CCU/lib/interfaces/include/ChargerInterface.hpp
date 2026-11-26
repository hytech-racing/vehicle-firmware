#ifndef CHARGERINTERFACE_H
#define CHARGERINTERFACE_H

/* ETL Library Includes */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Dependencies */
#include "CANInterface.h"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

#include <FlexCAN_T4.h>

namespace default_charger_params
{
    constexpr uint8_t BYTE_SHIFT = 8;
    constexpr uint8_t BYTE_MASK = 0xFF;
    constexpr float DATA_SCALAR = 10.0F; // charger reports values in units of 0.1
    constexpr uint16_t MAX_RAW_VALUE = 0xFFFF;

    /* Elcon Message 1 (CCU -> charger) control byte */
    constexpr uint8_t CONTROL_START_CHARGING = 0;
    constexpr uint8_t CONTROL_STOP_CHARGING = 1;

    /* Elcon Message 2 (charger broadcast) status flag bits */
    constexpr uint8_t FLAG_HARDWARE_FAILURE = (1U << 0);      // hardware failure
    constexpr uint8_t FLAG_OVER_TEMPERATURE = (1U << 1);      // over temperature protection
    constexpr uint8_t FLAG_INPUT_VOLTAGE_FAULT = (1U << 2);   // input voltage wrong, charger stops working
    constexpr uint8_t FLAG_BATTERY_NOT_DETECTED = (1U << 3);  // charger stays off (no battery / reverse polarity)
    constexpr uint8_t FLAG_COMMUNICATION_TIMEOUT = (1U << 4); // charger did not receive Message 1 in time
    constexpr uint8_t FAULT_FLAGS_MASK = FLAG_HARDWARE_FAILURE | FLAG_OVER_TEMPERATURE | FLAG_INPUT_VOLTAGE_FAULT | FLAG_COMMUNICATION_TIMEOUT;
}; // namespace default_charger_params

struct ChargerData_s
{
    uint8_t output_dc_voltage_high;
    uint8_t output_dc_voltage_low;
    uint8_t output_current_high;
    uint8_t output_current_low;
    uint8_t flags;
    uint8_t input_ac_voltage_high;
    uint8_t input_ac_voltage_low;
};

class ChargerInterface
{
  public:
    ChargerInterface() = default;

    void receiveChargerCANMsg(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Enqueue Elcon Message 1. The charger runs CC/CV on its own: it outputs up to max_current_A (CC)
     *        until its terminal voltage reaches max_voltage_V, then holds that voltage while current tapers (CV).
     * @param max_voltage_V CV setpoint (pack voltage)
     * @param max_current_A CC limit
     * @param enable true sends "start charging", false sends "stop charging"
     */
    void enqueueChargingCANMsg(float max_voltage_V, float max_current_A, bool enable);

    ChargerData_s get_latest_charger_data() { return _charger_data; };

    /* Charger values are sent as high/low bytes of a 16-bit int in units of 0.1 */
    float getOutputCurrentA() const { return _combineBytes(_charger_data.output_current_high, _charger_data.output_current_low); };
    float getDCOutputVoltageV() const { return _combineBytes(_charger_data.output_dc_voltage_high, _charger_data.output_dc_voltage_low); };
    float getACInputVoltageV() const { return _combineBytes(_charger_data.input_ac_voltage_high, _charger_data.input_ac_voltage_low); };

    /* Status flag decoding (Elcon Message 2, byte 5) */
    uint8_t getStatusFlags() const { return _charger_data.flags; };
    bool isHardwareFailure() const { return (_charger_data.flags & default_charger_params::FLAG_HARDWARE_FAILURE) != 0; };
    bool isOverTemperature() const { return (_charger_data.flags & default_charger_params::FLAG_OVER_TEMPERATURE) != 0; };
    bool isInputVoltageFault() const { return (_charger_data.flags & default_charger_params::FLAG_INPUT_VOLTAGE_FAULT) != 0; };
    bool isBatteryNotDetected() const { return (_charger_data.flags & default_charger_params::FLAG_BATTERY_NOT_DETECTED) != 0; };
    bool isCommunicationTimeout() const { return (_charger_data.flags & default_charger_params::FLAG_COMMUNICATION_TIMEOUT) != 0; };
    bool isFaulted() const { return (_charger_data.flags & default_charger_params::FAULT_FLAGS_MASK) != 0; };

  private:
    /**
     * @brief Combines the high/low bytes of the charger's 16-bit, 0.1-unit format into a float
     */
    float _combineBytes(uint8_t high, uint8_t low) const;

    /**
     * @brief Converts a value to the charger's 16-bit, 0.1-unit format (clamped to the valid range)
     */
    uint16_t _toChargerUnits(float value) const;

    ChargerData_s _charger_data = {};
};

using ChargerInterfaceInstance = etl::singleton<ChargerInterface>;

#endif /* CHARGERINTERFACE_H */
