#include "VCRCANInterfaceImpl.hpp"


void VCRCANInterfaceImpl::onRAUXCANReceive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().rear_aux_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::onINVERTERCANReceive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().inverter_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::onTELEMCANReceive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    VCRCANInterfaceInstance::instance().telem_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void VCRCANInterfaceImpl::receieveIDSwitch(CANInterfaces_s &interfaces,
                                        const CAN_message_t &msg,
                                        unsigned long curr_millis,
                                        CANInterfaceType_e interface_type
)
{
    switch (msg.id)
    {
        case PEDALS_SYSTEM_DATA_CANID:
        {
            interfaces.vcf_interface.receivePedalsCANMsg(msg, curr_millis);
            break;
        }
        case STEERING_DATA_CANID:
        {
            interfaces.vcf_interface.receiveSteeringCANMsg(msg, curr_millis);
            break;
        }
        case FRONT_SUSPENSION_CANID:
        {
            interfaces.vcf_interface.receiveFrontSuspensionCANMsg(msg, curr_millis);
            break;
        }
        case DASH_INPUT_CANID:
        {
            interfaces.vcf_interface.receiveDashboardCANMsg(msg, curr_millis);
            break;
        }
        case ACU_OK_CANID:
        {
            interfaces.acu_interface.receiveACUOKMessage(msg, curr_millis);
            break;
        }
        case EM_MEASUREMENT_CANID:
        {
            interfaces.acu_interface.receiveEMMeasurementMessage(msg, curr_millis);
            break;
        }
        case DRIVEBRAIN_TORQUE_LIM_INPUT_CANID:
        {
            if (interface_type == CANInterfaceType_e::RAUX)
            {
                interfaces.db_interface.receiveDrivebrainTorqueCommandRAUX(msg, curr_millis);
            }
            else if (interface_type == CANInterfaceType_e::TELEM)
            {
                interfaces.db_interface.receiveDrivebrainTorqueCommandTELEM(msg, curr_millis);
            }
            break;
        }
        case DRIVEBRAIN_SPEED_SET_INPUT_CANID:
        {
            if (interface_type == CANInterfaceType_e::RAUX)
            {
                interfaces.db_interface.receiveDrivebrainSpeedCommandRAUX(msg, curr_millis);
            }
            else if (interface_type == CANInterfaceType_e::TELEM)
            {
                interfaces.db_interface.receiveDrivebrainSpeedCommandTELEM(msg, curr_millis);
            }
            interfaces.db_interface.receiveDrivebrainSpeedCommandTELEM(msg, curr_millis);
            break;
        }

        // Front Left Inverter
        {
            case INV1_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_CONTROL(msg, curr_millis);
                break;
            }
            case INV1_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_ELEC(msg, curr_millis);
                break;
            }
            case INV1_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_ACTIVE_CURRENT(msg, curr_millis);
                break;
            }
            case INV1_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.fl_inverter_interface.receive_TEMP_AND_FAULT(msg, curr_millis);
                break;
            }
            case INV1_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.fl_inverter_interface.receive_FOC_CURRENTS(msg, curr_millis);
                break;
            }
            case INV1_STATUS_GENERAL_IO_CANID:
            {
                interfaces.fl_inverter_interface.receive_GENERAL_IO(msg, curr_millis);
                break;
            }
            case INV1_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_AC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
            case INV1_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.fl_inverter_interface.receive_DC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
        }

        // Front Right Inverter
        {
            case INV2_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_CONTROL(msg, curr_millis);
                break;
            }
            case INV2_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_ELEC(msg, curr_millis);
                break;
            }
            case INV2_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_ACTIVE_CURRENT(msg, curr_millis);
                break;
            }
            case INV2_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.fr_inverter_interface.receive_TEMP_AND_FAULT(msg, curr_millis);
                break;
            }
            case INV2_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.fr_inverter_interface.receive_FOC_CURRENTS(msg, curr_millis);
                break;
            }
            case INV2_STATUS_GENERAL_IO_CANID:
            {
                interfaces.fr_inverter_interface.receive_GENERAL_IO(msg, curr_millis);
                break;
            }
            case INV2_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_AC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
            case INV2_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.fr_inverter_interface.receive_DC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
        }


        // Rear Left Inverter
        {
            case INV3_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_CONTROL(msg, curr_millis);
                break;
            }
            case INV3_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_ELEC(msg, curr_millis);
                break;
            }
            case INV3_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_ACTIVE_CURRENT(msg, curr_millis);
                break;
            }
            case INV3_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.rl_inverter_interface.receive_TEMP_AND_FAULT(msg, curr_millis);
                break;
            }
            case INV3_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.rl_inverter_interface.receive_FOC_CURRENTS(msg, curr_millis);
                break;
            }
            case INV3_STATUS_GENERAL_IO_CANID:
            {
                interfaces.rl_inverter_interface.receive_GENERAL_IO(msg, curr_millis);
                break;
            }
            case INV3_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_AC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
            case INV3_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.rl_inverter_interface.receive_DC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
        }


        // Rear Right Inverter
        {
            case INV4_STATUS_GENERAL_CONTROL_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_CONTROL(msg, curr_millis);
                break;
            }
            case INV4_STATUS_GENERAL_ELEC_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_ELEC(msg, curr_millis);
                break;
            }
            case INV4_STATUS_ACTIVE_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_ACTIVE_CURRENT(msg, curr_millis);
                break;
            }
            case INV4_STATUS_TEMP_AND_FAULT_CANID:
            {
                interfaces.rr_inverter_interface.receive_TEMP_AND_FAULT(msg, curr_millis);
                break;
            }
            case INV4_STATUS_FOC_CURRENTS_CANID:
            {
                interfaces.rr_inverter_interface.receive_FOC_CURRENTS(msg, curr_millis);
                break;
            }
            case INV4_STATUS_GENERAL_IO_CANID:
            {
                interfaces.rr_inverter_interface.receive_GENERAL_IO(msg, curr_millis);
                break;
            }
            case INV4_STATUS_AC_CONFIG_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_AC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
            case INV4_STATUS_DC_CONFIG_CURRENT_CANID:
            {
                interfaces.rr_inverter_interface.receive_DC_CONFIG_CURRENT(msg, curr_millis);
                break;
            }
        }

        default:
        {
            break;
        }
    }
}

void VCRCANInterfaceImpl::sendAllCANMsgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
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

