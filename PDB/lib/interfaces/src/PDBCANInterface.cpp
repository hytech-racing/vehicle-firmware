#include "PDBCANInterface.hpp"


void PDBCAN::PDB_CAN_receive_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg)
{
    switch (msg.id)
    {
        case CAR_STATES_CANID:
        {
            interfaces.vcr.receiveVehicleStateCANMsg(msg);
            break;
        }
        default:
        {
            break;
        }
    }
}

void PDBCAN::onReceive(const CAN_message_t &msg)
{
    PDB_CAN_receive_switch(CANInterfacesInstance::instance(), msg);
}