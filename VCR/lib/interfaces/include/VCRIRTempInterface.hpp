#ifndef VCR_IR_TEMP_INTERFACE_HPP
#define VCR_IR_TEMP_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* Shared Interface */
#include "IRTempInterface.hpp"

/**
 * @brief Rear brake rotor IR temp sensors on VCR REAR_AUX. left_sensor = RL, right_sensor = RR
 */
class VCRIRTempInterface : public IRTempInterface
{
  public:
    VCRIRTempInterface()
        : IRTempInterface(ir_temp_default_params::RL_BASE_CAN_ID) {}
};

using VCRIRTempInterfaceInstance = etl::singleton<VCRIRTempInterface>;

#endif // VCR_IR_TEMP_INTERFACE_HPP
