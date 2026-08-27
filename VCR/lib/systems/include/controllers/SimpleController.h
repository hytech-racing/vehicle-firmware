#ifndef SIMPLECONTROLLER
#define SIMPLECONTROLLER

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"

namespace simple_tc_default_params
{
    /**
     *  @param REAR_TORQUE_SCALE 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = 50/50, 2 = RWD
     *  @param REAR_REGEN_TORQUE_SCALE same as rear_torque_scale but applies to regen torque split. 0 = all regen torque on fronts, 1 = 50/50, 2 = all regen torque on rears
     */
    constexpr float REAR_TORQUE_SCALE = 1.3f; // more torque to rears
    constexpr float REAR_REGEN_TORQUE_SCALE = 0.3f; // more regen torque to fronts
}

struct SimpleTCParams_s
{
    speed_rpm motor_max_rpm;
    torque_nm motor_max_torque_nm;
    torque_nm motor_max_regen_torque_nm;
    float rear_torque_scale;
    float rear_regen_torque_scale;
};

class SimpleTorqueController
{
public:


    /**
     * @brief This is our Simple Torque Controller (TC) which corresponds to Mode 0.
     *        This TC has tunable F/R torque balance, as well as accel/regen torque balance (tuned independently)
    */
    SimpleTorqueController(SimpleTCParams_s params)
        : _params {
            .motor_max_rpm = dti_motor_params::MOTOR_MAX_RPM,
            .motor_max_torque_nm = dti_motor_params::MOTOR_MAX_TORQUE_NM,
            .motor_max_regen_torque_nm = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM,
            .rear_torque_scale = simple_tc_default_params::REAR_TORQUE_SCALE,
            .rear_regen_torque_scale = simple_tc_default_params::REAR_REGEN_TORQUE_SCALE
        }
    {};

    /// @brief calculates torque output based off max torque and simple torque scaling
    DrivetrainCommand_s evaluate(const VCRData_s &state, unsigned long curr_millis);

private:

    SimpleTCParams_s _params;

};

#endif