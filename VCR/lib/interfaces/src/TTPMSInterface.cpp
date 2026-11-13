#include "TTPMSInterface.hpp"
#include "VCRCANInterfaceImpl.hpp"

#include <string.h>

namespace
{
    /* Datasheet: Big-Endian / Motorola byte order */
    uint16_t read_be_u16(const uint8_t *buf, size_t index)
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(buf[index]) << 8) | buf[index + 1]);
    }
}

TTPMSAllSensorData_s TTPMSInterface::getTTPMSData() const
{
    return _ttpms_data;
}

bool TTPMSInterface::setBaseCANID(uint16_t base_can_id)
{
    if (base_can_id < _params.min_base_can_id || base_can_id > _params.max_decode_base_can_id)
    {
        return false;
    }
    _base_can_id = base_can_id;
    _ttpms_data = {}; // old data belongs to the old ID layout
    return true;
}

uint16_t TTPMSInterface::getSensorBaseCANID(TTPMSCorner_e corner) const
{
    return static_cast<uint16_t>(_base_can_id + (static_cast<uint16_t>(corner) * ttpms_default_params::MSGS_PER_SENSOR));
}

bool TTPMSInterface::isTTPMSMessage(uint32_t can_id) const
{
    return (can_id >= _base_can_id) && (can_id < (static_cast<uint32_t>(_base_can_id) + ttpms_default_params::NUM_TTPMS_IDS));
}

bool TTPMSInterface::receiveTTPMSData(const CAN_message_t &can_msg, unsigned long curr_millis)
{
    if (can_msg.flags.extended || !isTTPMSMessage(can_msg.id) || can_msg.len < 8)
    {
        return false;
    }

    const uint32_t offset = can_msg.id - _base_can_id;
    const size_t sensor_index = offset / ttpms_default_params::MSGS_PER_SENSOR;
    const size_t msg_index = offset % ttpms_default_params::MSGS_PER_SENSOR;
    TTPMSSingleSensorData_s &sensor = _sensorAt(sensor_index);
    const uint8_t *buf = can_msg.buf;

    switch (msg_index)
    {
        case 0: // Serial number, battery voltage, HS pressure, gauge pressure
        {
            sensor.serial_number = read_be_u16(buf, 0);
            sensor.battery_voltage_mv = read_be_u16(buf, 2);
            sensor.pressure_psi = _hsPressureFromRaw(read_be_u16(buf, 4)) * ttpms_default_params::MBAR_TO_PSI;
            sensor.gauge_pressure_psi = static_cast<float>(read_be_u16(buf, 6)) * ttpms_default_params::MBAR_TO_PSI; // 1 mBar per bit
            sensor.last_pressure_recv_millis = curr_millis;
            break;
        }
        case 1: // IR CH1 - CH4
        case 2: // IR CH5 - CH8
        case 3: // IR CH9 - CH12
        case 4: // IR CH13 - CH16
        {
            const size_t first_channel = (msg_index - 1) * 4;
            for (size_t i = 0; i < 4; i++)
            {
                sensor.temp_data[first_channel + i] = _tempFromRaw(read_be_u16(buf, i * 2));
            }
            sensor.last_temp_recv_millis = curr_millis;
            break;
        }
        case 5: // Transmission count, RSSI, sensor temperature, node ID
        {
            sensor.transmission_count = read_be_u16(buf, 0);
            sensor.rssi_dbm = static_cast<int16_t>(read_be_u16(buf, 2)); // signed
            sensor.sensor_temp = _tempFromRaw(read_be_u16(buf, 4));
            sensor.node_id = read_be_u16(buf, 6);
            sensor.last_status_recv_millis = curr_millis;
            break;
        }
        default:
        {
            return false;
        }
    }

    _updateHealth(sensor);
    return true;
}

bool TTPMSInterface::isSensorActive(const TTPMSSingleSensorData_s &sensor, unsigned long curr_millis, unsigned long timeout_ms) const
{
    return (sensor.last_pressure_recv_millis != 0) && ((curr_millis - sensor.last_pressure_recv_millis) <= timeout_ms);
}

bool TTPMSInterface::buildBaseConfigMsg(uint16_t target_can_id, uint16_t new_base_id, TTPMSBitRate_e bit_rate,
                                        TTPMSEmissivityMode_e emissivity_mode, CAN_message_t &msg_out) const
{
    uint8_t bit_rate_raw = static_cast<uint8_t>(bit_rate);
    uint8_t emissivity_raw = static_cast<uint8_t>(emissivity_mode);
    if (target_can_id > 0x7FF ||
        new_base_id < _params.min_base_can_id || new_base_id > _params.max_receiver_base_can_id ||
        bit_rate_raw < static_cast<uint8_t>(TTPMSBitRate_e::BITRATE_1MBPS) ||
        bit_rate_raw > static_cast<uint8_t>(TTPMSBitRate_e::BITRATE_100KBPS) ||
        emissivity_raw > static_cast<uint8_t>(TTPMSEmissivityMode_e::CUSTOM))
    {
        return false;
    }

    _packCommandID(msg_out, target_can_id, _params.command_ids.base_config);
    msg_out.buf[2] = static_cast<uint8_t>(new_base_id >> 8);
    msg_out.buf[3] = static_cast<uint8_t>(new_base_id & 0xFF);
    msg_out.buf[4] = _params.sensor_assignment_default;
    msg_out.buf[5] = bit_rate_raw;
    msg_out.buf[6] = emissivity_raw;
    msg_out.buf[7] = 0;
    return true;
}

bool TTPMSInterface::buildRFConfigMsg(uint16_t target_can_id, uint8_t network_id, uint8_t node_id,
                                      uint32_t frequency_hz, CAN_message_t &msg_out) const
{
    if (target_can_id > 0x7FF ||
        frequency_hz < _params.min_rf_frequency_hz || frequency_hz > _params.max_rf_frequency_hz ||
        (frequency_hz % _params.rf_frequency_resolution_hz) != 0)
    {
        return false;
    }

    uint16_t frequency_raw = static_cast<uint16_t>(frequency_hz / _params.rf_frequency_resolution_hz);
    _packCommandID(msg_out, target_can_id, _params.command_ids.rf_config);
    msg_out.buf[2] = network_id;
    msg_out.buf[3] = node_id;
    msg_out.buf[4] = static_cast<uint8_t>(frequency_raw >> 8);
    msg_out.buf[5] = static_cast<uint8_t>(frequency_raw & 0xFF);
    msg_out.buf[6] = 0;
    msg_out.buf[7] = 0;
    return true;
}

bool TTPMSInterface::buildEmissivityConfigMsgs(uint16_t target_can_id,
                                               const std::array<float, ttpms_default_params::TEMP_CHANNELS> &emissivity,
                                               std::array<CAN_message_t, 3> &msgs_out) const
{
    if (target_can_id > 0x7FF)
    {
        return false;
    }

    std::array<uint8_t, ttpms_default_params::TEMP_CHANNELS> emissivity_raw;
    for (size_t i = 0; i < ttpms_default_params::TEMP_CHANNELS; i++)
    {
        // Small tolerance so values like 0.01f / 1.0f are not rejected by float rounding
        if (emissivity[i] < (_params.min_emissivity - 0.001f) || emissivity[i] > (_params.max_emissivity + 0.001f))
        {
            return false;
        }
        emissivity_raw[i] = static_cast<uint8_t>((emissivity[i] * 100.0f) + 0.5f); // 0.01 per bit
    }

    // T1 - T6
    _packCommandID(msgs_out[0], target_can_id, _params.command_ids.emissivity_t1_t6);
    for (size_t i = 0; i < 6; i++)
    {
        msgs_out[0].buf[2 + i] = emissivity_raw[i];
    }

    // T7 - T12
    _packCommandID(msgs_out[1], target_can_id, _params.command_ids.emissivity_t7_t12);
    for (size_t i = 0; i < 6; i++)
    {
        msgs_out[1].buf[2 + i] = emissivity_raw[6 + i];
    }

    // T13 - T16, then the default emissivity which must not be changed
    _packCommandID(msgs_out[2], target_can_id, _params.command_ids.emissivity_t13_t16);
    for (size_t i = 0; i < 4; i++)
    {
        msgs_out[2].buf[2 + i] = emissivity_raw[12 + i];
    }
    msgs_out[2].buf[6] = _params.emissivity_raw_default;
    msgs_out[2].buf[7] = 0;
    return true;
}

bool TTPMSInterface::startConfigSequence(const CAN_message_t *msgs, size_t num_msgs, unsigned long curr_millis)
{
    if (_config_active || msgs == nullptr || num_msgs == 0 || num_msgs > ttpms_default_params::MAX_CONFIG_MSGS)
    {
        return false;
    }

    for (size_t i = 0; i < num_msgs; i++)
    {
        _config_msgs[i] = msgs[i];
    }
    _num_config_msgs = num_msgs;
    _config_active = true;
    _config_start_millis = curr_millis;
    _last_config_send_millis = curr_millis - _params.config_send_period_ms; // send on first update
    return true;
}

void TTPMSInterface::updateConfigSequence(unsigned long curr_millis)
{
    if (!_config_active)
    {
        return;
    }

    if ((curr_millis - _config_start_millis) > _params.config_send_duration_ms)
    {
        // Datasheet: do NOT continuously send CAN messages to the receiver. Power cycle it now to apply.
        _config_active = false;
        _num_config_msgs = 0;
        return;
    }

    if ((curr_millis - _last_config_send_millis) >= _params.config_send_period_ms)
    {
        for (size_t i = 0; i < _num_config_msgs; i++)
        {
            uint8_t buf[sizeof(CAN_message_t)] = {};
            memmove(buf, &_config_msgs[i], sizeof(CAN_message_t));
            VCRCANInterfaceInstance::instance().rear_aux_can_tx_buffer.push_back(buf, sizeof(CAN_message_t));
        }
        _last_config_send_millis = curr_millis;
    }
}

TTPMSSingleSensorData_s &TTPMSInterface::_sensorAt(size_t index)
{
    switch (index)
    {
        case 0: return _ttpms_data.fl_ttpms;
        case 1: return _ttpms_data.fr_ttpms;
        case 2: return _ttpms_data.rl_ttpms;
        default: return _ttpms_data.rr_ttpms;
    }
}

void TTPMSInterface::_updateHealth(TTPMSSingleSensorData_s &sensor) const
{
    // Only flag once the relevant message has actually been received
    sensor.is_battery_low = (sensor.last_pressure_recv_millis != 0) && (sensor.battery_voltage_mv < _params.low_battery_threshold_mv);
    sensor.is_rssi_weak = (sensor.last_status_recv_millis != 0) && (sensor.rssi_dbm < _params.min_rssi_dbm);
}

float TTPMSInterface::_hsPressureFromRaw(uint16_t raw) const
{
    // Datasheet specifies HS pressure as signed (0.1 mBar/bit, +3000 mBar offset).
    // Reading it unsigned would decode absolute pressures below 3000 mBar as ~9000+ mBar.
    return (static_cast<float>(static_cast<int16_t>(raw)) * _params.hs_pressure_scale_mbar) + _params.hs_pressure_offset_mbar;
}

float TTPMSInterface::_tempFromRaw(uint16_t raw) const
{
    return (static_cast<float>(raw) * _params.temp_scale_degc) + _params.temp_offset_degc;
}

void TTPMSInterface::_packCommandID(CAN_message_t &msg, uint16_t target_can_id, uint16_t command_id) const
{
    msg = CAN_message_t();
    msg.id = target_can_id;
    msg.len = 8;
    msg.flags.extended = 0;
    msg.buf[0] = static_cast<uint8_t>(command_id >> 8);  // Big-endian / Motorola
    msg.buf[1] = static_cast<uint8_t>(command_id & 0xFF);
}
