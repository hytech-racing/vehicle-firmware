#include "VCFCANInterfaceImpl.hpp"


void VCFCANInterfaceImpl::on_telem_can_recv(const CAN_message_t& msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    VCFCANInterfaceInstance::instance().telem_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void VCFCANInterfaceImpl::on_front_aux_can_recv(const CAN_message_t& msg)
{
    // VCFCANInterfaceInstance::instance().TELEM_CAN.write(msg); //immediately forward onto telem can to view data
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    VCFCANInterfaceInstance::instance().front_aux_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void VCFCANInterfaceImpl::vcf_recv_switch(CANInterfaces_s& interfaces, const CAN_message_t& msg, uint32_t millis, CANInterfaceType_e interface_type)
{
    switch (msg.id)
    {
    case DASHBOARD_BUZZER_CONTROL_CANID:
    {
        interfaces.vcr_interface.receiveDashboardControlData(msg);
        break;
    }
    case BMS_VOLTAGES_CANID:
    {
        interfaces.acu_interface.receiveACUVoltages(msg);
        break;
    }
    case ACU_OK_CANID:
    {
        interfaces.dash_interface.receiveACUOKCANMsg(msg);
        break;
    }
    case CAR_STATES_CANID:
    {
        interfaces.vcr_interface.recieveCarStatesCANMsg(msg);
        // VCR sends car states on TELEM. Forward them unchanged onto FRONT_AUX for FDC.
        if (interface_type == CANInterfaceType_e::TELEM)
        {
            std::array<uint8_t, CAN_MSG_SIZE> buf;
            memmove(buf.data(), &msg, CAN_MSG_SIZE);
            VCFCANInterfaceInstance::instance().front_aux_can_tx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
        }
        break;
    }
    case INV1_STATUS_CANID:
    {
        interfaces.vcr_interface.receiveINV1StatusCANMsg(msg);
        break;
    }
    case INV2_STATUS_CANID:
    {
        interfaces.vcr_interface.receiveINV2StatusCANMsg(msg);
        break;
    }
    case INV3_STATUS_CANID:
    {
        interfaces.vcr_interface.receiveINV3StatusCANMsg(msg);
        break;
    }
    case INV4_STATUS_CANID:
    {
        interfaces.vcr_interface.receiveINV4StatusCANMsg(msg);
        break;
    }
    // FDC sends EBS data on FRONT_AUX. Forward it unchanged onto TELEM so it's logged.
    case EBS_DATA_CANID:
    {
        if (interface_type == CANInterfaceType_e::FAUX)
        {
            std::array<uint8_t, CAN_MSG_SIZE> buf;
            memmove(buf.data(), &msg, CAN_MSG_SIZE);
            VCFCANInterfaceInstance::instance().telem_can_tx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
        }
        break;
    }
    // Front IR temp sensors are on FRONT_AUX. Forward them unchanged onto TELEM so they're logged.
    case FL_IR_TEMP_CH1_CH4_CANID:
    case FL_IR_TEMP_CH5_CH8_CANID:
    case FL_IR_TEMP_CH9_CH12_CANID:
    case FL_IR_TEMP_CH13_CH16_CANID:
    case FR_IR_TEMP_CH1_CH4_CANID:
    case FR_IR_TEMP_CH5_CH8_CANID:
    case FR_IR_TEMP_CH9_CH12_CANID:
    case FR_IR_TEMP_CH13_CH16_CANID:
    {
        if (interface_type == CANInterfaceType_e::FAUX)
        {
            interfaces.ir_temp_interface.receiveIRTempData(msg, millis);

            std::array<uint8_t, CAN_MSG_SIZE> buf;
            memmove(buf.data(), &msg, CAN_MSG_SIZE);
            VCFCANInterfaceInstance::instance().telem_can_tx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

void VCFCANInterfaceImpl::send_all_CAN_msgs(CANTXBuffer_t& buffer, FlexCAN_T4_Base* can_interface)
{
    CAN_message_t msg;
    while (buffer.available())
    {
        std::array<uint8_t, CAN_MSG_SIZE> buf;
        buffer.pop_front(buf.data(), CAN_MSG_SIZE);
        memmove(&msg, buf.data(), CAN_MSG_SIZE);
        can_interface->write(msg);
    }
}
