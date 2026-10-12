#ifndef PDB_CAN_INTERFACE_H
#define PDB_CAN_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "ht_can.h"
#include "STM32_CANInterface.hpp"

/* Local Interface Includes */
#include "VCRInterface.hpp"


struct CANInterfaces_s
{
    explicit CANInterfaces_s(VCRInterface &vcr_int) : vcr(vcr_int) {}

    VCRInterface &vcr;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

namespace PDBCAN
{
    void PDB_CAN_receive_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg);

    /**
     * @brief Receive callback registered with the shared STM32CANInterface; routes each frame to
     *        PDB_CAN_receive_switch(). Runs in interrupt context.
    */
    void onReceive(const CAN_message_t &msg);
}

#endif