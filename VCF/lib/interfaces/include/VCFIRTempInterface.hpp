#ifndef VCF_IR_TEMP_INTERFACE_HPP
#define VCF_IR_TEMP_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* Shared Interface */
#include "IRTempInterface.hpp"

/**
 * @brief Front brake rotor IR temp sensors on VCF FRONT_AUX. left_sensor = FL, right_sensor = FR
 */
class VCFIRTempInterface : public IRTempInterface
{
  public:
    VCFIRTempInterface()
        : IRTempInterface(ir_temp_default_params::FL_BASE_CAN_ID)
    {
    }
};

using VCFIRTempInterfaceInstance = etl::singleton<VCFIRTempInterface>;

#endif // VCF_IR_TEMP_INTERFACE_HPP
