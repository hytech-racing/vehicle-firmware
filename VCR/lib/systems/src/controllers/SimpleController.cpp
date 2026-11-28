#include "controllers/SimpleController.hpp"


DrivetrainCommand_s TorqueControllerSimple::evaluate(const VCRData_s& state, unsigned long curr_millis)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accel_request goes between 1.0 and -1.0
    DrivetrainCommand_s out;

    float accel_request = state.interface_data.recvd_pedals_data.pedals_data.accel_percent - state.interface_data.recvd_pedals_data.pedals_data.brake_percent;
    float torque_request = 0.0f;

    constexpr float BALANCE = 2.0;

    if (accel_request >= 0.0)
    {
        // Positive torque request
        torque_request = accel_request * _params.amk_max_torque;

        out.desired_speeds = {_params.amk_max_rpm, _params.amk_max_rpm, _params.amk_max_rpm, _params.amk_max_rpm};
        out.torque_limits.fl = torque_request * (BALANCE - _params.rear_torque_scale);
        out.torque_limits.fr = torque_request * (BALANCE - _params.rear_torque_scale);
        out.torque_limits.rl = torque_request * _params.rear_torque_scale;
        out.torque_limits.rr = torque_request * _params.rear_torque_scale;
    }
    else
    {
        // regen torque request
        torque_request = _params.amk_max_regen_torque * accel_request * -1.0; // NOLINT (-1 is magic number)

        out.desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

        out.torque_limits.fl = torque_request * (BALANCE - _params.rear_regen_torque_scale);
        out.torque_limits.fr = torque_request * (BALANCE - _params.rear_regen_torque_scale);
        out.torque_limits.rl = torque_request * _params.rear_regen_torque_scale;
        out.torque_limits.rr = torque_request * _params.rear_regen_torque_scale;
    }

    return out;
}
