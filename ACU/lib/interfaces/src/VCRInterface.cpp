#include "VCRInterface.hpp"

#include "CoreCANInterface.hpp"


void VCRInterface::set_monitoring_data(bool imd_ok, bool bms_ok, bool latch_ok)
{
    _curr_data.imd_ok = imd_ok;
    _curr_data.bms_ok = bms_ok;
    _curr_data.latch_ok = latch_ok;
}

void VCRInterface::handle_enqueue_acu_ok_CAN_message()
{
    ACU_OK_t msg = {};
    msg.imd_ok = _curr_data.imd_ok;
    msg.bms_ok = _curr_data.bms_ok;
    msg.latch_ok = _curr_data.latch_ok;
    CAN_message_t msg_packed;
    msg_packed.id = Pack_ACU_OK_ht_can(&msg, msg_packed.buf, &msg_packed.len, (uint8_t*)&msg_packed.flags.extended);
    ACUCANInterfaceInstance::instance().enqueueOnSwitch(ACUCANBusses_e::CCU, msg_packed);
}
