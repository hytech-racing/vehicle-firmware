#include "controllers/SimpleController.h"


DrivetrainCommand_s SimpleTorqueController::evaluate(const VCRData_s &state, unsigned long curr_millis)
{
    const PedalsSystemData_s &pedals_data = state.interface_data.recvd_pedals_data.pedals_data;

    DrivetrainCommand_s out = { .control_mode = _params.control_mode,
                                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;
    constexpr float scale_range = 2.0f;

    if (_params.control_mode == DrivetrainControlMode_e::TORQUE)
    {
        float torque_request = 0.0f;

        if (accel_request >= 0.0f)
        {
            torque_request = accel_request * _params.motor_max_torque_nm;

            out.desired_torques.FL = torque_request * (scale_range - _params.rear_torque_scale);
            out.desired_torques.FR = torque_request * (scale_range - _params.rear_torque_scale);
            out.desired_torques.RL = torque_request * _params.rear_torque_scale;
            out.desired_torques.RR = torque_request * _params.rear_torque_scale;
        }
        else
        {
            /**
             * @note acccel_request is negative here, motor_max_regen_torque_nm is positive, so torque_request
             *       comes out negative naturally. Sign just signals regen/braking, matching how InverterInterface
             *       routes negative torque to SET_BRAKE_CURRENT.
            */
            torque_request = _params.motor_max_regen_torque_nm * accel_request;

            out.desired_torques.FL = torque_request * (scale_range - _params.rear_regen_torque_scale);
            out.desired_torques.FR = torque_request * (scale_range - _params.rear_regen_torque_scale);
            out.desired_torques.RL = torque_request * _params.rear_regen_torque_scale;
            out.desired_torques.RR = torque_request * _params.rear_regen_torque_scale;
        }
    }
    else   // SPEED CONTROL
    {
        float speed_request = accel_request * _params.motor_max_rpm;

        out.desired_speeds.FL = speed_request;
        out.desired_speeds.FR = speed_request;
        out.desired_speeds.RL = speed_request;
        out.desired_speeds.RR = speed_request;
    }

    return out;
}