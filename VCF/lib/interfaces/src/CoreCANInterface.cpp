#include "CoreCANInterface.hpp"


void VCFCANInterface::enqueueSwitch(VCFCANBusses_e bus, const CAN_message_t& msg)
{
    switch (bus)
    {
    case VCFCANBusses_e::TELEM:
    {
        _telem_bus.enqueueOnBus(msg);
        break;
    }
    case VCFCANBusses_e::FRONT_AUX:
    {
        _front_aux_bus.enqueueOnBus(msg);
        break;
    }
    }
}

void VCFCANInterface::processAllRxBuffers()
{
    auto handler = CANBus<CAN1>::RecvHandler_t::create<VCFCANInterface, &VCFCANInterface::_recvSwitch>(*this);
    _telem_bus.processRxBuffer(handler);
    _front_aux_bus.processRxBuffer(handler);
}

void VCFCANInterface::sendAllTxBuffers()
{
    _telem_bus.sendTxBuffer();
    _front_aux_bus.sendTxBuffer();
}

void VCFCANInterface::_recvSwitch(const CAN_message_t& msg)
{
    switch (msg.id)
    {
    case DASHBOARD_BUZZER_CONTROL_CANID:
    {
        _vcr_interface.receiveDashboardControlData(msg);
        break;
    }
    case BMS_VOLTAGES_CANID:
    {
        _acu_interface.receiveACUVoltages(msg);
        break;
    }
    case ACU_OK_CANID:
    {
        _dash_interface.receiveACUOKCANMsg(msg);
        break;
    }
    case CAR_STATES_CANID:
    {
        _vcr_interface.recieveCarStatesCANMsg(msg);
        break;
    }
    case INV1_STATUS_CANID:
    {
        _vcr_interface.receiveINV1StatusCANMsg(msg);
        break;
    }
    case INV2_STATUS_CANID:
    {
        _vcr_interface.receiveINV2StatusCANMsg(msg);
        break;
    }
    case INV3_STATUS_CANID:
    {
        _vcr_interface.receiveINV3StatusCANMsg(msg);
        break;
    }
    case INV4_STATUS_CANID:
    {
        _vcr_interface.receiveINV4StatusCANMsg(msg);
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
    {
        _brake_rotor_temp_interface.receiveBrakeRotorTempCANMsgs(msg);
        break;
    }
    default:
    {
        break;
    }
    }
}
