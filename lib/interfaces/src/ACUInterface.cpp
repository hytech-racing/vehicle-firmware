#include "ACUInterface.h"


void ACUInterface::receive_acu_ok_message(const CAN_message_t &msg)
{
    ACU_OK_t unpacked_msg;
    Unpack_ACU_OK_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT (implicitly decay pointer)

    _acu_data.bms_ok = unpacked_msg.bms_ok;
    _acu_data.imd_ok = unpacked_msg.imd_ok;
}

void ACUInterface::receive_acu_voltages(const CAN_message_t &msg)
{
    BMS_VOLTAGES_t unpacked_msg;
    Unpack_BMS_VOLTAGES_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT (implicitly decay pointer)

    _acu_data.pack_voltage = HYTECH_total_pack_voltage_ro_fromS(unpacked_msg.total_pack_voltage_ro);
    _acu_data.min_cell_voltage = HYTECH_min_cell_voltage_ro_fromS(unpacked_msg.min_cell_voltage_ro);
}