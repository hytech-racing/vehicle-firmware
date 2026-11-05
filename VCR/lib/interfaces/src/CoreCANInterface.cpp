#include "CoreCANInterface.hpp"

#include "SystemTimeInterface.h"


void VCRCANInterface::enqueueSwitch(VCRCANBusses_e bus, const CAN_message_t& msg)
{
    switch (bus)
    {
    case VCRCANBusses_e::TELEM:
    {
        _telem_bus.enqueueOnBus(msg);
        break;
    }
    case VCRCANBusses_e::INVERTER:
    {
        _inverter_bus.enqueueOnBus(msg);
        break;
    }
    }
}

void VCRCANInterface::processAllRxBuffers()
{
    _inverter_bus.processRxBuffer(CANBus<CAN3>::RecvHandler_t::create<VCRCANInterface, &VCRCANInterface::_onInverterBusRecv>(*this));
    _telem_bus.processRxBuffer(CANBus<CAN1>::RecvHandler_t::create<VCRCANInterface, &VCRCANInterface::_onTelemBusRecv>(*this));
}

void VCRCANInterface::sendAllTxBuffers()
{
    _inverter_bus.sendTxBuffer();
    _telem_bus.sendTxBuffer();
}

void VCRCANInterface::_onTelemBusRecv(const CAN_message_t& msg)
{
    _recvSwitch(msg, VCRCANBusses_e::TELEM);
}

void VCRCANInterface::_onInverterBusRecv(const CAN_message_t& msg)
{
    _recvSwitch(msg, VCRCANBusses_e::INVERTER);
}

void VCRCANInterface::_recvSwitch(const CAN_message_t& msg, VCRCANBusses_e bus)
{
    const unsigned long millis = sys_time::hal_millis();
    switch (msg.id)
    {
    case PEDALS_SYSTEM_DATA_CANID:
    {
        _vcf_interface.receive_pedals_message(msg, millis);
        break;
    }
    case STEERING_DATA_CANID:
    {
        _vcf_interface.receive_steering_message(msg, millis);
        break;
    }
    case FRONT_SUSPENSION_CANID:
    {
        _vcf_interface.receive_front_suspension_message(msg, millis);
        break;
    }
    case DASH_INPUT_CANID:
    {
        _vcf_interface.receive_dashboard_message(msg, millis);
        break;
    }
    case ACU_OK_CANID:
    {
        _acu_interface.receive_acu_ok_message(msg, millis);
        break;
    }
    case EM_MEASUREMENT_CANID:
    {
        _acu_interface.receive_em_measurement(msg, millis);
        break;
    }

    case DRIVEBRAIN_TORQUE_LIM_INPUT_CANID:
    {
        // TODO: route to receive_drivebrain_torque_lim_command_auxillary() once rear aux is brought up
        if (bus == VCRCANBusses_e::TELEM)
        {
            _db_interface.receive_drivebrain_torque_lim_command_telem(msg, millis);
        }
        break;
    }
    case DRIVEBRAIN_SPEED_SET_INPUT_CANID:
    {
        // TODO: route to receive_drivebrain_speed_command_auxillary() once rear aux is brought up
        // NOTE: carried over as-is: the telem receive runs twice for TELEM messages, and once for any other bus
        if (bus == VCRCANBusses_e::TELEM)
        {
            _db_interface.receive_drivebrain_speed_command_telem(msg, millis);
        }
        _db_interface.receive_drivebrain_speed_command_telem(msg, millis);
        break;
    }

        // Front Left Inverter
        {
        case INV1_STATUS_CANID:
        {
            _fl_inverter_interface.receive_INV_STATUS(msg, millis);
            break;
        }
        case INV1_TEMPS_CANID:
        {
            _fl_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV1_DYNAMICS_CANID:
        {
            _fl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV1_POWER_CANID:
        {
            _fl_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV1_FEEDBACK_CANID:
        {
            _fl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
        }

        // Front right inverter
        {
        case INV2_STATUS_CANID:
        {
            _fr_inverter_interface.receive_INV_STATUS(msg, millis);
            break;
        }
        case INV2_TEMPS_CANID:
        {
            _fr_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV2_DYNAMICS_CANID:
        {
            _fr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV2_POWER_CANID:
        {
            _fr_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV2_FEEDBACK_CANID:
        {
            _fr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
        }

        // Rear left inverter
        {
        case INV3_STATUS_CANID:
        {
            _rl_inverter_interface.receive_INV_STATUS(msg, millis);
            break;
        }
        case INV3_TEMPS_CANID:
        {
            _rl_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV3_DYNAMICS_CANID:
        {
            _rl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV3_POWER_CANID:
        {
            _rl_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV3_FEEDBACK_CANID:
        {
            _rl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
        }

        // Rear right inverter
        {
        case INV4_STATUS_CANID:
        {
            _rr_inverter_interface.receive_INV_STATUS(msg, millis);
            break;
        }
        case INV4_TEMPS_CANID:
        {
            _rr_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV4_DYNAMICS_CANID:
        {
            _rr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV4_POWER_CANID:
        {
            _rr_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV4_FEEDBACK_CANID:
        {
            _rr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
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
    {
        break;
    }
    }
}
