#include "ChargerInterface.hpp"

#include "CCUCANInterfaceImpl.hpp" // this needs to fixed at some point


void ChargerInterface::receiveChargerCANMsg(const CAN_message_t& msg, unsigned long curr_millis)
{
    CHARGER_DATA_t charger_data_msg;
    Unpack_CHARGER_DATA_ht_can(&charger_data_msg, &msg.buf[0], msg.len);
    _charger_data.output_dc_voltage_high = charger_data_msg.output_dc_voltage_high;
    _charger_data.output_dc_voltage_low = charger_data_msg.output_dc_voltage_low;
    _charger_data.output_current_high = charger_data_msg.output_current_high;
    _charger_data.output_current_low = charger_data_msg.output_current_low;
    _charger_data.flags = charger_data_msg.flags;
    _charger_data.input_ac_voltage_high = charger_data_msg.input_ac_voltage_high;
    _charger_data.input_ac_voltage_low = charger_data_msg.input_ac_voltage_low;
}

void ChargerInterface::enqueueChargingCANMsg(float max_voltage_V, float max_current_A, bool enable)
{
    // Voltage and current are 16-bit ints in units of 0.1, sent as high byte then low byte.
    // e.g. 520.0 V -> 5200 -> 0x1450 -> high = 0x14, low = 0x50
    const uint16_t voltage_raw = enable ? _toChargerUnits(max_voltage_V) : 0;
    const uint16_t current_raw = enable ? _toChargerUnits(max_current_A) : 0;

    CHARGER_CONTROL_t charger_control = {};
    charger_control.max_charging_voltage_high = static_cast<uint8_t>(voltage_raw >> default_charger_params::BYTE_SHIFT);
    charger_control.max_charging_voltage_low = static_cast<uint8_t>(voltage_raw & default_charger_params::BYTE_MASK);
    charger_control.max_charging_current_high = static_cast<uint8_t>(current_raw >> default_charger_params::BYTE_SHIFT);
    charger_control.max_charging_current_low = static_cast<uint8_t>(current_raw & default_charger_params::BYTE_MASK);
    charger_control.control = enable ? default_charger_params::CONTROL_START_CHARGING : default_charger_params::CONTROL_STOP_CHARGING;
    CAN_util::enqueue_msg(&charger_control, &Pack_CHARGER_CONTROL_ht_can, CCUCANInterfaceInstance::instance().charger_can_tx_buffer);
}

float ChargerInterface::_combineBytes(uint8_t high, uint8_t low) const
{
    return static_cast<float>((high << default_charger_params::BYTE_SHIFT) | low) / default_charger_params::DATA_SCALAR;
}

uint16_t ChargerInterface::_toChargerUnits(float value) const
{
    float scaled = value * default_charger_params::DATA_SCALAR;
    if (scaled <= 0.0F)
    {
        return 0;
    }
    if (scaled >= static_cast<float>(default_charger_params::MAX_RAW_VALUE))
    {
        return default_charger_params::MAX_RAW_VALUE;
    }
    return static_cast<uint16_t>(scaled + 0.5F); // round to nearest
}
