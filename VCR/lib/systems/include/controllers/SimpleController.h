#ifndef SIMPLECONTROLLER
#define SIMPLECONTROLLER

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"


namespace simple_tc_default_params
{
    /**
     * @param ACCEL_BIAS is a fraction (0.0 to 1.0)
     *
     *   0.0 = FWD (all torque to front)
     *   0.5 = AWD (balanced)
     *   1.0 = RWD (all torque to rear)
     *
     * @param REGEN_BIAS is a fraction (0.0 to 1.0). Works the same as ACCEL_BIAS, just for regen
    */
    constexpr float ACCEL_BIAS = 0.65f;  // rear-biased under acceleration, more torque to rears
    constexpr float REGEN_BIAS = 0.15f;  // front-biased under regen/braking, more torque to fronts

    /// @note Set to SPEED to command speed instead of torque.
    constexpr DrivetrainControlMode_e SIMPLE_CONTROLLER_MODE = DrivetrainControlMode_e::TORQUE;
}

struct SimpleTCParams_s
{
    speed_rpm motor_max_rpm;
    torque_nm motor_max_torque_nm;
    torque_nm motor_max_regen_torque_nm;
    float accel_bias;
    float regen_bias;
    DrivetrainControlMode_e control_mode;
};

class SimpleTorqueController
{
public:

    /**
     * @brief This is our Simple Torque Controller (TC) which corresponds to Mode 0
     * @note 1) This TC has tunable front/rear torque bias during both acceleration and regen (tuned independently)
     *       2) Left/right split for fronts and rears is always symmetric.
     *       3) Can choose to command either torque or speed
    */
    explicit SimpleTorqueController(SimpleTCParams_s params)
        : _params(params)
    {}

    /// @brief Default constructor
    SimpleTorqueController()
        : _params {
            .motor_max_rpm = dti_motor_params::MOTOR_MAX_RPM,
            .motor_max_torque_nm = dti_motor_params::MOTOR_MAX_TORQUE_NM,
            .motor_max_regen_torque_nm = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM,
            .accel_bias = simple_tc_default_params::ACCEL_BIAS,
            .regen_bias = simple_tc_default_params::REGEN_BIAS,
            .control_mode = simple_tc_default_params::SIMPLE_CONTROLLER_MODE
        }
    {}

    /// @brief calculates torque or speed output (per params.control_mode) using a front/rear bias fraction
    DrivetrainCommand_s evaluate(const VCRData_s &state, unsigned long curr_millis);

private:

    SimpleTCParams_s _params;

};

#endif