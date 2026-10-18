#include "CCUCANInterfaceImpl.hpp"


void CCUCANInterfaceImpl::onACUCANReceive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    CCUCANInterfaceInstance::instance().acu_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void CCUCANInterfaceImpl::onChargerCANReceive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    CCUCANInterfaceInstance::instance().charger_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void CCUCANInterfaceImpl::CCUReceiveIDSwitch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type)
{
    switch (msg.id)
    {
        case BMS_VOLTAGES_CANID:
        {
            interfaces.acu_interface.receiveBMSVoltagesCANMsg(msg, millis);
            break;
        }
        case BMS_DETAILED_VOLTAGES_CANID:
        {
            interfaces.acu_interface.receiveBMSDetailedVoltagesCANMsg(msg, millis);
            break;
        }
        case BMS_STATUS_CANID:
        {
            interfaces.acu_interface.receiveBMSStatusCANMsg(msg, millis);
            break;
        }
        case CHARGER_DATA_CANID:
        {
            interfaces.charger_interface.receiveChargerCANMsg(msg, millis, interfaces.acu_interface, interfaces.max_pack_voltage, interfaces.cell_cutoff_voltage);
            break;
        }
        case BMS_TEMPS_CANID:
        {
            interfaces.acu_interface.receiveBMSTempsCANMsg(msg, millis);
            break;
        }
        case BMS_DETAILED_TEMPS_CANID:
        {
            interfaces.acu_interface.receiveBMSDetailedTempsCANMsg(msg, millis);
            break;
        }
        case BMS_BOARD_DETAILED_TEMPS_CANID:
        {
            interfaces.acu_interface.receiveBMSBoardTemps(msg, millis);
            break;
        }
        case EM_MEASUREMENT_CANID:
        {
            interfaces.em_interface.receiveEMMeasurmentCANMsg(msg, millis);
            break;
        }
        case STATE_OF_CHARGE_CANID:
        {
            interfaces.acu_interface.receiveStateOfChargeCANMSG(msg, millis);
            break;
        }
        default:
        {
            break;
        }
    }
}

void CCUCANInterfaceImpl::clearAllCANBuffers(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
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
