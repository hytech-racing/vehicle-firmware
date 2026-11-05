#include "CoreCANInterface.hpp"

#include "SystemTimeInterface.h"


void ACUCANInterface::enqueueSwitch(ACUCANBusses_e bus, const CAN_message_t& msg)
{
    switch (bus)
    {
    case ACUCANBusses_e::CCU:
    {
        _ccu_bus.enqueueOnBus(msg);
        break;
    }
    case ACUCANBusses_e::EM:
    {
        _em_bus.enqueueOnBus(msg);
        break;
    }
    }
}

void ACUCANInterface::processAllRxBuffers()
{
    _ccu_bus.processRxBuffer(CANBus<CAN2>::RecvHandler_t::create<ACUCANInterface, &ACUCANInterface::_recvSwitch>(*this));
    _em_bus.processRxBuffer(CANBus<CAN3>::RecvHandler_t::create<ACUCANInterface, &ACUCANInterface::_onEMBusRecv>(*this));
}

void ACUCANInterface::sendAllTxBuffers()
{
    _ccu_bus.sendTxBuffer();
    _em_bus.sendTxBuffer();
}

void ACUCANInterface::_onEMBusRecv(const CAN_message_t& msg)
{
    _ccu_bus.enqueueOnBus(msg);
    _recvSwitch(msg);
}

void ACUCANInterface::_recvSwitch(const CAN_message_t& msg)
{
    const unsigned long millis = sys_time::hal_millis();
    switch (msg.id)
    {
    case CCU_STATUS_CANID:
    {
        _ccu_interface.receive_CCU_status_message(msg, millis);
        break;
    }
    case EM_MEASUREMENT_CANID:
    {
        _em_interface.receive_EM_measurement_message(msg, millis);
        break;
    }
    default:
    {
        break;
    }
    }
}
