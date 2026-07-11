#include "ACUInterface.h"
#include "VCFCANInterfaceImpl.h"

void ACUInterface::receive_ACU_voltages(const CAN_message_t &can_msg)
{
    BMS_VOLTAGES_t unpacked_msg;
    Unpack_BMS_VOLTAGES_hytech(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT (implicitly decay pointer)
    _min_cell_voltage = HYTECH_min_cell_voltage_ro_fromS(unpacked_msg.min_cell_voltage_ro);
}
