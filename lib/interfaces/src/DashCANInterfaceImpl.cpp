#include "DashCANInterfaceImpl.h"


void DashCAN::dash_CAN_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis)
{
    switch (msg.id)
    {
        case BMS_VOLTAGES_CANID:
        {
            interfaces.acu_interface.receive_acu_voltages(msg);
            break;
        }
        case ACU_OK_CANID:
        {
            interfaces.acu_interface.receive_acu_ok_message(msg);
            break;
        }
        case PEDALS_SYSTEM_DATA_CANID:
        {
            interfaces.vcf_interface.receive_pedals_message(msg, millis);
            break;
        }
        case INV1_DYNAMICS_CANID:
        {
            interfaces.vcr_interface.receive_inv_dynamics(msg, millis);
            break;
        }
        case CAR_STATES_CANID:
        {
            interfaces.vcr_interface.receive_vehicle_state(msg);
            break;
        }
        case DASH_INPUT_CANID:
        {
            interfaces.vcf_interface.receive_dashboard_message(msg, millis);
            break;
        }
        case INV1_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_1(msg);
            break;
        }
        case INV2_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_2(msg);
            break;
        }
        case INV3_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_3(msg);
            break;
        }
        case INV4_STATUS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_status_4(msg);
            break;
        }
        case INV1_TEMPS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_temperature_1(msg);
            break;
        }
        case INV2_TEMPS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_temperature_2(msg);
            break;
        }
        case INV3_TEMPS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_temperature_3(msg);
            break;
        }
        case INV4_TEMPS_CANID:
        {
            interfaces.vcr_interface.receive_inverter_temperature_4(msg);
            break;
        }
        default:
        {
            break;
        }
    }
}