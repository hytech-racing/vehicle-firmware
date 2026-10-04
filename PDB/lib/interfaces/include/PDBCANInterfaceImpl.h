#ifndef PDBCANINTERFACEIMPL_H
#define PDBCANINTERFACEIMPL_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "hytech.h"

#include "VCRInterface.h"

struct CANInterfaces_s {
    explicit CANInterfaces_s(VCRInterface &vcr_int) : vcr(vcr_int) {}
    // Personal notes since used from Dashboard (pretty neat implementation)
    //Explicit ensures CAN_Interfaces are properly declared and casted, rather than just initializing with a given VCR interface
    //Notation: VCR... &VCR (used in a declaration) means it is a reference to a VCRInterface object, when used as &vcr alone then that is an address
    //Done this way with an initializer list since the moment you reach { the struct must be initialized
    //Initializing a plain vcr would create a second duplicate vcr interface than point to the same vcr interface object

    VCRInterface &vcr;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

namespace PDBCAN {
    void PDB_CAN_receive_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg);
}

#endif