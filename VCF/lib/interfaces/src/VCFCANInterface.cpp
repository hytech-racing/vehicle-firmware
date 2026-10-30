#include "VCFCANInterface.hpp"


void VCFCANInterface::init(uint32_t telem_baudrate, uint32_t front_aux_baudrate)
{
    handle_CAN_setup(_telem_can, telem_baudrate, &VCFCANInterface::_onTelemCANRecv);
    handle_CAN_setup(_front_aux_can, front_aux_baudrate, &VCFCANInterface::_onFrontAuxCANRecv);
}

void VCFCANInterface::enqueue(VCFCANBus_e bus, const CAN_message_t& msg)
{
    std::array<uint8_t, _CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, _CAN_MSG_SIZE);
    switch (bus)
    {
    case VCFCANBus_e::TELEM:
    {
        _telem_can_tx_buffer.push_back(buf.data(), _CAN_MSG_SIZE);
        break;
    }
    case VCFCANBus_e::FRONT_AUX:
    {
        _front_aux_can_tx_buffer.push_back(buf.data(), _CAN_MSG_SIZE);
        break;
    }
    }
}

void VCFCANInterface::processAllRxBuffers(unsigned long curr_millis)
{
    _processRxBuffer(_telem_can_rx_buffer, curr_millis, CANInterfaceType_e::TELEM);
    _processRxBuffer(_front_aux_can_rx_buffer, curr_millis, CANInterfaceType_e::FAUX);
}

void VCFCANInterface::sendAllTxBuffers()
{
    _sendAllCANMsgs(_telem_can_tx_buffer, _telem_can);
    _sendAllCANMsgs(_front_aux_can_tx_buffer, _front_aux_can);
}

void VCFCANInterface::_onTelemCANRecv(const CAN_message_t& msg)
{
    _pushMsg(VCFCANInterfaceInstance::instance()._telem_can_rx_buffer, msg);
}

void VCFCANInterface::_onFrontAuxCANRecv(const CAN_message_t& msg)
{
    _pushMsg(VCFCANInterfaceInstance::instance()._front_aux_can_rx_buffer, msg);
}

void VCFCANInterface::_pushMsg(CANRXBuffer_t& buffer, const CAN_message_t& msg)
{
    std::array<uint8_t, _CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, _CAN_MSG_SIZE);
    buffer.push_back(buf.data(), _CAN_MSG_SIZE);
}

void VCFCANInterface::_sendAllCANMsgs(CANTXBuffer_t& buffer, FlexCAN_T4_Base& can_interface)
{
    CAN_message_t msg;
    while (buffer.available())
    {
        std::array<uint8_t, _CAN_MSG_SIZE> buf;
        buffer.pop_front(buf.data(), _CAN_MSG_SIZE);
        memmove(&msg, buf.data(), _CAN_MSG_SIZE);
        can_interface.write(msg);
    }
}

void VCFCANInterface::_processRxBuffer(CANRXBuffer_t& buffer, unsigned long curr_millis, CANInterfaceType_e interface_type)
{
    while (buffer.available())
    {
        CAN_message_t recvd_msg;
        std::array<uint8_t, _CAN_MSG_SIZE> buf;
        buffer.pop_front(buf.data(), _CAN_MSG_SIZE);
        memmove(&recvd_msg, buf.data(), _CAN_MSG_SIZE);
        _recvSwitch(recvd_msg, curr_millis, interface_type);
    }
}

void VCFCANInterface::_recvSwitch(const CAN_message_t& msg, unsigned long curr_millis, CANInterfaceType_e interface_type)
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
