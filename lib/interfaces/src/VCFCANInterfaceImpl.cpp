#include "VCFCANInterfaceImpl.h"

void VCFCANInterfaceImpl::on_telem_can_recv(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    VCFCANInterfaceInstance::instance().telem_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void VCFCANInterfaceImpl::on_front_aux_can_recv(const CAN_message_t &msg)
{
    // VCFCANInterfaceInstance::instance().TELEM_CAN.write(msg); //immediately forward onto telem can to view data
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    VCFCANInterfaceInstance::instance().front_aux_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void VCFCANInterfaceImpl::vcf_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type)
{
    switch (msg.id)
    {
        case DASHBOARD_BUZZER_CONTROL_CANID:
        {
            interfaces.vcr_interface.receive_dash_control_data(msg);
            break;
        }
        case BMS_VOLTAGES_CANID:
        {
            interfaces.acu_interface.receive_ACU_voltages(msg);
            break;
        }
        case ACU_OK_CANID:
        {
            interfaces.dash_interface.receive_ACU_OK(msg);
            break;
        }
        case CAR_STATES_CANID:
        {
            interfaces.vcr_interface.receive_car_states_data(msg);
            break;
        }
        case INV1_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_1(msg);
            break;
        }
        case INV2_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_2(msg);
            break;
        }
        case INV3_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_3(msg);
            break;
        }
        case INV4_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_4(msg);
            break;
        }
        case FL_BRAKE_ROTOR_SENSOR_TEMP_CANID:
        case FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID:
        case FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID:
        case FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID:
        case FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID:
        case FR_BRAKE_ROTOR_SENSOR_TEMP_CANID:
        case FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID:
        case FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID:
        case FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID:
        case FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID:
            interfaces.brake_rotor_temp_interface.receiveBrakeRotorTempData(msg);
            break;
        default:
        {
            break;
        }
    }

}

void VCFCANInterfaceImpl::send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
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

