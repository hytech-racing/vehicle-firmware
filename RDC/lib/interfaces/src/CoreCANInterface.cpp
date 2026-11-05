#include "CoreCANInterface.hpp"

#include "SystemTimeInterface.h"


void RDCCANInterface::enqueueSwitch(RDCCANBusses_e bus, const CAN_message_t& msg)
{
    switch (bus)
    {
    case RDCCANBusses_e::DRIVERLESS:
    {
        _driverless_bus.enqueueOnBus(msg);
        break;
    }
    }
}

void RDCCANInterface::processAllRxBuffers()
{
    _driverless_bus.processRxBuffer(CANBus<CAN1>::RecvHandler_t::create<RDCCANInterface, &RDCCANInterface::_recvSwitch>(*this));
}

void RDCCANInterface::sendAllTxBuffers()
{
    _driverless_bus.sendTxBuffer();
}

void RDCCANInterface::_recvSwitch(const CAN_message_t& msg)
{
    const unsigned long millis = sys_time::hal_millis();
    switch (msg.id)
    {
        // TODO do we need this lowk we don't need this
    // case DRIVERLESS_STATUS_AND_STARTUP_CANID:
    // {
    //     _db_interface.receiveDriverlessState(msg, millis);
    //     break;
    // }
    case FDC_DRIVERLESS_MISSION_CANID:
    {
        _fdc_interface.receiveDVMission(msg, millis);
        break;
    }
    case EBS_DATA_CANID:
    {
        _fdc_interface.receiveEbsPressure(msg, millis);
        break;
    }
    case BRAKE_PRESSURE_CANID:
    {
        _fdc_interface.receiveBrakeData(msg, millis);
        break;
    }
    case CAR_STATES_CANID:
    {
        _db_interface.receiveVehicleState(msg, millis);
        break;
    }
    case RSS_STATUS_CANID:
    {
        _rss_interface.receiveRSSStatusCANMsg(msg, millis);
        break;
    }
    case RSS_BOOT_UP_CANID:
    {
        _rss_interface.receiveRSSBootCANmsg(msg, millis);
        break;
    }
    default:
        break;
    }
}
