#include "PDBCANInterfaceImpl.h"


void PDBCAN::PDB_CAN_receive_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg) {
    switch (msg.id) {
        case CAR_STATES_CANID:
        {
            interfaces.vcr.receive_vehicle_state(msg);
            break;
        }
        default:
        {
            break;
        }
    }
}