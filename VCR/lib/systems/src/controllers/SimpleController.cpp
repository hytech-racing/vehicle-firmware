#include "controllers/SimpleController.h"


DrivetrainCommand_s SimpleTorqueController::evaluate(const VCRData_s &state, unsigned long curr_millis)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    DrivetrainCommand_s out = { .torque_setpoints = {0.0f, 0.0f, 0.0f, 0.0f}};

    float accel_request = state.interface_data.recvd_pedals_data.pedals_data.accel_percent - state.interface_data.recvd_pedals_data.pedals_data.brake_percent;
    float torque_request = 0.0f;

    constexpr float scale_range = 2.0;

    if (accel_request >= 0.0)
    {
        torque_request = accel_request * _params.motor_max_rpm;

        out.torque_setpoints.FL = torque_request * (scale_range - _params.rear_torque_scale);
        out.torque_setpoints.FR = torque_request * (scale_range - _params.rear_torque_scale);
        out.torque_setpoints.RL = torque_request * _params.rear_torque_scale;
        out.torque_setpoints.RR = torque_request * _params.rear_torque_scale;
    }
    else
    {
        // Torque is negated here so the sign itself signals "this is regen/braking".
        // This is consistent with how DrivetrainSystem/InverterInterface route negative torque to SET_BRAKE_CURRENT.
        torque_request = -1.0f * (_params.motor_max_regen_torque_nm * accel_request * -1.0f);

        out.torque_setpoints.FL = torque_request * (scale_range - _params.rear_regen_torque_scale);
        out.torque_setpoints.FR = torque_request * (scale_range - _params.rear_regen_torque_scale);
        out.torque_setpoints.RL = torque_request * _params.rear_regen_torque_scale;
        out.torque_setpoints.RR = torque_request * _params.rear_regen_torque_scale;
    }

    return out;
}