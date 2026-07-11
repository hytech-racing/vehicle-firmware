#include "ACUInterface.h"
#include "VCRCANInterfaceImpl.h"

void ACUInterface::receive_acu_ok_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    ACU_OK_t acu_msg = {};
    Unpack_ACU_OK_hytech(&acu_msg, &msg.buf[0], msg.len);
    _curr_data.imd_ok = acu_msg.imd_ok;
    _curr_data.bms_ok = acu_msg.bms_ok;
    if (_curr_data.last_recv_millis == 0) { _received_first_acu_heartbeat = true; }
    _curr_data.last_recv_millis = curr_millis;
}

void ACUInterface::receive_em_measurement(const CAN_message_t &msg, unsigned long curr_millis)
{
    EM_MEASUREMENT_t em_msg = {};
    Unpack_EM_MEASUREMENT_hytech(&em_msg, &msg.buf[0], msg.len);
    _curr_data.em_current = HYTECH_em_current_ro_fromS(em_msg.em_current_ro);
    _curr_data.em_voltage = HYTECH_em_voltage_ro_fromS(em_msg.em_voltage_ro);
}

ACUCANInterfaceData_s ACUInterface::get_latest_data(uint64_t curr_millis)
{
    if(_received_first_acu_heartbeat || _curr_data.heartbeat_ok)
    {
        _received_first_acu_heartbeat = false;
        _curr_data.heartbeat_ok = ((curr_millis - _curr_data.last_recv_millis) < _max_heartbeat_interval_ms);
    }
    else
    {
        _curr_data.heartbeat_ok = false;
    }
    return _curr_data;
}