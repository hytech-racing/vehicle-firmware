#include "VCRCANInterfaceImpl.h"


void VCRCANInterfaceImpl::on_auxillary_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().rear_aux_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::on_inverter_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().inverter_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::on_telem_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().telem_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::vcr_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type)
{
    switch (msg.id)
    {
        case PEDALS_SYSTEM_DATA_CANID:
        {
            interfaces.vcf_interface.receive_pedals_message(msg, millis);
            break;
        }
        case STEERING_DATA_CANID:
        {
            interfaces.vcf_interface.receive_steering_message(msg, millis);
            break;
        }
        case FRONT_SUSPENSION_CANID:
        {
            interfaces.vcf_interface.receive_front_suspension_message(msg, millis);
            break;
        }
        case DASH_INPUT_CANID:
        {
            interfaces.vcf_interface.receive_dashboard_message(msg, millis);
            break;
        }
        case ACU_OK_CANID:
        {
            interfaces.acu_interface.receive_acu_ok_message(msg, millis);
            break;
        }
        case EM_MEASUREMENT_CANID:
        {
            interfaces.acu_interface.receive_em_measurement(msg, millis);
            break;
        }
        case DRIVEBRAIN_TORQUE_LIM_INPUT_CANID:
        {
            if (interface_type == CANInterfaceType_e::RAUX)
            {
                interfaces.db_interface.receive_drivebrain_torque_lim_command_auxillary(msg, millis);
            }
            else if (interface_type == CANInterfaceType_e::TELEM)
            {
                interfaces.db_interface.receive_drivebrain_torque_lim_command_telem(msg, millis);
            }
            break;
        }
        case DRIVEBRAIN_SPEED_SET_INPUT_CANID:
        {
            if (interface_type == CANInterfaceType_e::RAUX)
            {
                interfaces.db_interface.receive_drivebrain_speed_command_auxillary(msg, millis);
            }
            else if (interface_type == CANInterfaceType_e::TELEM)
            {
                interfaces.db_interface.receive_drivebrain_speed_command_telem(msg, millis);
            }
            interfaces.db_interface.receive_drivebrain_speed_command_telem(msg, millis);
            break;
        }

        // Front Left Inverter
        {
            case INV1_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_CONTROL(msg, millis);
                break;
            }
            case INV1_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_ELEC(msg, millis);
                break;
            }
            case INV1_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_ACTIVE_CURRENT(msg, millis);
                break;
            }
            case INV1_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.fl_inverter_interface.receive_TEMP_AND_FAULT(msg, millis);
                break;
            }
            case INV1_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.fl_inverter_interface.receive_FOC_CURRENTS(msg, millis);
                break;
            }
            case INV1_STATUS_GENERAL_IO_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_IO(msg, millis);
                break;
            }
            case INV1_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_AC_CONFIG_CURRENT(msg, millis);
                break;
            }
            case INV1_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_DC_CONFIG_CURRENT(msg, millis);
                break;
            }
        }

        // Front Right Inverter
        {
            case INV2_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_CONTROL(msg, millis);
                break;
            }
            case INV2_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_ELEC(msg, millis);
                break;
            }
            case INV2_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_ACTIVE_CURRENT(msg, millis);
                break;
            }
            case INV2_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.fr_inverter_interface.receive_TEMP_AND_FAULT(msg, millis);
                break;
            }
            case INV2_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.fr_inverter_interface.receive_FOC_CURRENTS(msg, millis);
                break;
            }
            case INV2_STATUS_GENERAL_IO_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_IO(msg, millis);
                break;
            }
            case INV2_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_AC_CONFIG_CURRENT(msg, millis);
                break;
            }
            case INV2_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_DC_CONFIG_CURRENT(msg, millis);
                break;
            }
        }


        // Rear Left Inverter
        {
            case INV3_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_CONTROL(msg, millis);
                break;
            }
            case INV3_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_ELEC(msg, millis);
                break;
            }
            case INV3_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_ACTIVE_CURRENT(msg, millis);
                break;
            }
            case INV3_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.rl_inverter_interface.receive_TEMP_AND_FAULT(msg, millis);
                break;
            }
            case INV3_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.rl_inverter_interface.receive_FOC_CURRENTS(msg, millis);
                break;
            }
            case INV3_STATUS_GENERAL_IO_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_IO(msg, millis);
                break;
            }
            case INV3_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_AC_CONFIG_CURRENT(msg, millis);
                break;
            }
            case INV3_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_DC_CONFIG_CURRENT(msg, millis);
                break;
            }
        }


        // Rear Right Inverter
        {
            case INV4_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_CONTROL(msg, millis);
                break;
            }
            case INV4_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_ELEC(msg, millis);
                break;
            }
            case INV4_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_ACTIVE_CURRENT(msg, millis);
                break;
            }
            case INV4_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.rr_inverter_interface.receive_TEMP_AND_FAULT(msg, millis);
                break;
            }
            case INV4_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.rr_inverter_interface.receive_FOC_CURRENTS(msg, millis);
                break;
            }
            case INV4_STATUS_GENERAL_IO_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_IO(msg, millis);
                break;
            }
            case INV4_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_AC_CONFIG_CURRENT(msg, millis);
                break;
            }
            case INV4_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_DC_CONFIG_CURRENT(msg, millis);
                break;
            }
        }

        default:
        {
            break;
        }
    }
}

void VCRCANInterfaceImpl::send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
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

