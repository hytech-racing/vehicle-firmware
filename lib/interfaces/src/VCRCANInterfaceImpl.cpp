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

void vcr_CAN_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type)
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
            case INV1_STATUS_CANID:
            {
                interfaces.fl_inverter_interface.receive_INV_STATUS(msg, millis);
                break;
            }
            case INV1_TEMPS_CANID:
            {
                interfaces.fl_inverter_interface.receive_INV_TEMPS(msg, millis);
                break;
            }
            case INV1_DYNAMICS_CANID:
            {
                interfaces.fl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
                break;
            }
            case INV1_POWER_CANID:
            {
                interfaces.fl_inverter_interface.receive_INV_POWER(msg, millis);
                break;
            }
            case INV1_FEEDBACK_CANID:
            {
                interfaces.fl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
                break;
            }
        }

        // Front right inverter
        {
            case INV2_STATUS_CANID:
            {
                interfaces.fr_inverter_interface.receive_INV_STATUS(msg, millis);
                break;
            }
            case INV2_TEMPS_CANID:
            {
                interfaces.fr_inverter_interface.receive_INV_TEMPS(msg, millis);
                break;
            }
            case INV2_DYNAMICS_CANID:
            {
                interfaces.fr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
                break;
            }
            case INV2_POWER_CANID:
            {
                interfaces.fr_inverter_interface.receive_INV_POWER(msg, millis);
                break;
            }
            case INV2_FEEDBACK_CANID:
            {
                interfaces.fr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
                break;
            }
        }

        // Rear left inverter
        {
            case INV3_STATUS_CANID:
            {
                interfaces.rl_inverter_interface.receive_INV_STATUS(msg, millis);
                break;
            }
            case INV3_TEMPS_CANID:
            {
                interfaces.rl_inverter_interface.receive_INV_TEMPS(msg, millis);
                break;
            }
            case INV3_DYNAMICS_CANID:
            {
                interfaces.rl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
                break;
            }
            case INV3_POWER_CANID:
            {
                interfaces.rl_inverter_interface.receive_INV_POWER(msg, millis);
                break;
            }
            case INV3_FEEDBACK_CANID:
            {
                interfaces.rl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
                break;
            }
        }

        // Rear right inverter
        {
            case INV4_STATUS_CANID:
            {
                interfaces.rr_inverter_interface.receive_INV_STATUS(msg, millis);
                break;
            }
            case INV4_TEMPS_CANID:
            {
                interfaces.rr_inverter_interface.receive_INV_TEMPS(msg, millis);
                break;
            }
            case INV4_DYNAMICS_CANID:
            {
                interfaces.rr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
                break;
            }
            case INV4_POWER_CANID:
            {
                interfaces.rr_inverter_interface.receive_INV_POWER(msg, millis);
                break;
            }
            case INV4_FEEDBACK_CANID:
            {
                interfaces.rr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
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

