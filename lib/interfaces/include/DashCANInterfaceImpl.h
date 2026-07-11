#ifndef DASHCANINTERFACEIMPL_H
#define DASHCANINTERFACEIMPL_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "hytech.h"

/* Local Interface Includes */
#include "ACUInterface.h"
#include "CANInterface.h"
#include "VCFInterface.h"
#include "VCRInterface.h"


struct CANInterfaces_s
{
    explicit CANInterfaces_s(VCFInterface &vcf_int,
                        ACUInterface &acu_int,
                        VCRInterface &vcr_int
    ) : vcf_interface(vcf_int),
        acu_interface(acu_int),
        vcr_interface(vcr_int)
    {};

    VCFInterface &vcf_interface;
    ACUInterface &acu_interface;
    VCRInterface &vcr_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;


namespace DashCAN
{
    void dash_CAN_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis);

    void write_CAN();
}
#endif