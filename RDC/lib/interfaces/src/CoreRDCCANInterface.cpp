#include "CoreRDCCANInterface.hpp"
#include "FlexCAN_T4.h"
#include "SystemTimeInterface.h"
#include "ht_can.h"

void RDCCANInterfaceImpl::RDCRecvSwitch(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type)
{
    switch (msg.id)
    {
        // TODO do we need this lowk we don't need this
    // case DRIVERLESS_STATUS_AND_STARTUP_CANID:
    // {
    //     interfaces.db_interface.receiveDriverlessState(msg, millis);
    //     break;
    // }
    case FDC_DRIVERLESS_MISSION_CANID:
    {
        interfaces.fdc_interface.receiveDVMission(msg, millis);
        break;
    }
    case EBS_DATA_CANID:
    {
        interfaces.fdc_interface.receiveEbsPressure(msg, millis);
        break;
    }
    case BRAKE_PRESSURE_CANID:
    {
        interfaces.fdc_interface.receiveBrakeData(msg, millis);
        break;
    }
    case CAR_STATES_CANID:
    {
        interfaces.db_interface.receiveVehicleState(msg, millis);
        break;
    }
    case RSS_STATUS_CANID:
    {
        interfaces.rss_interface.receiveRSSStatusCANMsg(msg, millis);
        break;
    }
    case RSS_BOOT_UP_CANID:
    {
        interfaces.rss_interface.receiveRSSBootCANmsg(msg, millis);
        break;
    }
    default:
        break;
    }
}

void RDCCANInterfaceImpl::onDVCANRecvHelper(const CAN_message_t &msg)
{
    CoreRDCCANInterfaceInstance::instance().onDVCANRecv(msg);
}

void CoreRDCCANInterface::onDVCANRecv(const CAN_message_t &msg)
{
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    memmove(buf.data(), &msg, sizeof(msg)); // NOLINT
    _driverless_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void CoreRDCCANInterface::processCANMessages(
    CANInterfaces_s can_interfaces,
    unsigned long curr_millis,
    etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_switch,
    CANInterfaceType_e interface_type)
{
    process_ring_buffer(_driverless_can_rx_buffer, can_interfaces, curr_millis, recv_switch, interface_type);
}
void CoreRDCCANInterface::sendAllCanMsgs()
{
    while (_driverless_can_tx_buffer.available())
    {
        CAN_message_t msg;
        std::array<uint8_t, sizeof(CAN_message_t)> buf;
        _driverless_can_tx_buffer.pop_front(buf.data(), sizeof(CAN_message_t));
        memmove(&msg, buf.data(), sizeof(msg)); // NOLINT
        DRIVERLESS_CAN.write(msg);
    }
}
