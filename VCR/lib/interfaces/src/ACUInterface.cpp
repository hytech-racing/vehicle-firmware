#include "ACUInterface.hpp"

#include "VCRCANInterfaceImpl.hpp"


void ACUInterface::receiveACUOKMessage(const CAN_message_t& msg, unsigned long curr_millis)
{
    ACU_OK_t acu_msg = {};
    Unpack_ACU_OK_ht_can(&acu_msg, &msg.buf[0], msg.len);

    _curr_data.is_imd_ok = acu_msg.imd_ok;
    _curr_data.is_bms_ok = acu_msg.bms_ok;
    _curr_data.is_latch_ok = acu_msg.latch_ok;

    if (_curr_data.last_acu_ok_recv_millis == 0)
    {
        _has_received_first_acu_heartbeat = true;
    }

    _curr_data.last_acu_ok_recv_millis = curr_millis;
}

void ACUInterface::receiveEMMeasurementMessage(const CAN_message_t& msg, unsigned long curr_millis)
{
    EM_MEASUREMENT_t em_msg = {};
    Unpack_EM_MEASUREMENT_ht_can(&em_msg, &msg.buf[0], msg.len);

    _curr_data.em_current = HT_CAN_em_current_ro_fromS(em_msg.em_current_ro);
    _curr_data.em_voltage = HT_CAN_em_voltage_ro_fromS(em_msg.em_voltage_ro);
}

void ACUInterface::receiveBMSVoltagesMessage(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_VOLTAGES_t voltages_msg = {};
    Unpack_BMS_VOLTAGES_ht_can(&voltages_msg, &msg.buf[0], msg.len);

    _curr_data.pack_voltage = HT_CAN_total_pack_voltage_ro_fromS(static_cast<float>(voltages_msg.total_pack_voltage_ro));
    _curr_data.last_bms_voltages_recv_millis = curr_millis;
}

ACUCANInterfaceData_s ACUInterface::getLatestData(uint64_t curr_millis)
{
    // ACU_OK heartbeat holds/latches
    if (_has_received_first_acu_heartbeat || _curr_data.is_acu_ok_heartbeat_ok)
    {
        _has_received_first_acu_heartbeat = false;
        _curr_data.is_acu_ok_heartbeat_ok = ((curr_millis - _curr_data.last_acu_ok_recv_millis) < _max_acu_ok_heartbeat_interval_ms);
    }
    else
    {
        _curr_data.is_acu_ok_heartbeat_ok = false;
    }

    // BMS_VOLTAGES heartbeat does not hold/latch, it recovers as soon as messages resume
    _curr_data.is_bms_voltages_heartbeat_ok =
        (_curr_data.last_bms_voltages_recv_millis != 0) && ((curr_millis - _curr_data.last_bms_voltages_recv_millis) < _max_bms_voltages_heartbeat_interval_ms);

    return _curr_data;
}