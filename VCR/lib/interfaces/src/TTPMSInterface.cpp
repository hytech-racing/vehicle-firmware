#include "TTPMSInterface.hpp"

TTPMSAllSensorData_s TTPMSInterface::getTTPMSData() const
{
    return _ttpms_data;
}

uint16_t TTPMSInterface::getSensorBaseCANID(TTPMSCorner_e corner) const
{
    return static_cast<uint16_t>(ttpms_default_params::BASE_CAN_ID + (static_cast<uint16_t>(corner) * ttpms_default_params::MSGS_PER_SENSOR));
}

bool TTPMSInterface::isTTPMSMessage(uint32_t can_id) const
{
    return (can_id >= ttpms_default_params::BASE_CAN_ID) && (can_id < (ttpms_default_params::BASE_CAN_ID + ttpms_default_params::NUM_TTPMS_IDS));
}

bool TTPMSInterface::receiveTTPMSData(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    if (can_msg.flags.extended || !isTTPMSMessage(can_msg.id) || can_msg.len < 8)
    {
        return false;
    }

    const uint32_t offset = can_msg.id - ttpms_default_params::BASE_CAN_ID;
    const size_t sensor_index = offset / ttpms_default_params::MSGS_PER_SENSOR;
    const size_t msg_index = offset % ttpms_default_params::MSGS_PER_SENSOR;
    TTPMSSingleSensorData_s& sensor = _sensorAt(sensor_index);
    const uint8_t* buf = can_msg.buf; // values are 16-bit big-endian (high byte first)

    switch (msg_index)
    {
    case 0: // Serial number, battery voltage, HS pressure, gauge pressure
    {
        sensor.serial_number = (buf[0] << 8) | buf[1];
        sensor.battery_voltage_mv = (buf[2] << 8) | buf[3];
        sensor.pressure_psi = _hsPressureFromRaw((buf[4] << 8) | buf[5]) * ttpms_default_params::MBAR_TO_PSI;
        sensor.gauge_pressure_psi = static_cast<float>((buf[6] << 8) | buf[7]) * ttpms_default_params::MBAR_TO_PSI; // 1 mBar per bit
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
            sensor.temp_data[first_channel + i] = _tempFromRaw((buf[i * 2] << 8) | buf[i * 2 + 1]);
        }
        sensor.last_temp_recv_millis = curr_millis;
        break;
    }
    case 5: // Transmission count, RSSI, sensor temperature, node ID
    {
        sensor.transmission_count = (buf[0] << 8) | buf[1];
        sensor.rssi_dbm = static_cast<int16_t>((buf[2] << 8) | buf[3]); // signed
        sensor.sensor_temp = _tempFromRaw((buf[4] << 8) | buf[5]);
        sensor.node_id = (buf[6] << 8) | buf[7];
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

bool TTPMSInterface::isSensorActive(const TTPMSSingleSensorData_s& sensor, unsigned long curr_millis, unsigned long timeout_ms) const
{
    return (sensor.last_pressure_recv_millis != 0) && ((curr_millis - sensor.last_pressure_recv_millis) <= timeout_ms);
}

TTPMSSingleSensorData_s& TTPMSInterface::_sensorAt(size_t index)
{
    switch (index)
    {
    case 0:
    {
        return _ttpms_data.fl_ttpms;
    }
    case 1:
    {
        return _ttpms_data.fr_ttpms;
    }
    case 2:
    {
        return _ttpms_data.rl_ttpms;
    }
    default:
    {
        return _ttpms_data.rr_ttpms;
    }
    }
}

void TTPMSInterface::_updateHealth(TTPMSSingleSensorData_s& sensor) const
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
