#include "CoreCANInterface.hpp"

#include "SystemTimeInterface.h"


void CCUCANInterface::enqueueSwitch(CCUCANBusses_e bus, const CAN_message_t& msg)
{
    switch (bus)
    {
    case CCUCANBusses_e::ACU:
    {
        _acu_bus.enqueueOnBus(msg);
        break;
    }
    case CCUCANBusses_e::CHARGER:
    {
        _charger_bus.enqueueOnBus(msg);
        break;
    }
    }
}

void CCUCANInterface::processAllRxBuffers()
{
    auto handler = CANBus<CAN1>::RecvHandler_t::create<CCUCANInterface, &CCUCANInterface::_recvSwitch>(*this);
    _acu_bus.processRxBuffer(handler);
    _charger_bus.processRxBuffer(handler);
}

void CCUCANInterface::sendAllTxBuffers()
{
    _acu_bus.sendTxBuffer();
    _charger_bus.sendTxBuffer();
}

void CCUCANInterface::_recvSwitch(const CAN_message_t& msg)
{
    const unsigned long millis = sys_time::hal_millis();
    switch (msg.id)
    {
    case BMS_VOLTAGES_CANID:
    {
        _acu_interface.receiveBMSVoltagesCANMsg(msg, millis);
        break;
    }
    case BMS_DETAILED_VOLTAGES_CANID:
    {
        _acu_interface.receiveBMSDetailedVoltagesCANMsg(msg, millis);
        break;
    }
    case BMS_STATUS_CANID:
    {
        _acu_interface.receiveBMSStatusCANMsg(msg, millis);
        break;
    }
    case CHARGER_DATA_CANID:
    {
        _charger_interface.receiveChargerCANMsg(msg, millis, _acu_interface, _max_pack_voltage, _cell_cutoff_voltage);
        break;
    }
    case BMS_TEMPS_CANID:
    {
        _acu_interface.receiveBMSTempsCANMsg(msg, millis);
        break;
    }
    case BMS_DETAILED_TEMPS_CANID:
    {
        _acu_interface.receiveBMSDetailedTempsCANMsg(msg, millis);
        break;
    }
    case BMS_BOARD_DETAILED_TEMPS_CANID:
    {
        _acu_interface.receiveBMSBoardTemps(msg, millis);
        break;
    }
    case EM_MEASUREMENT_CANID:
    {
        _em_interface.receiveEMMeasurmentCANMsg(msg, millis);
        break;
    }
    case STATE_OF_CHARGE_CANID:
    {
        _acu_interface.receiveStateOfChargeCANMSG(msg, millis);
        break;
    }
    default:
    {
        break;
    }
    }
}
