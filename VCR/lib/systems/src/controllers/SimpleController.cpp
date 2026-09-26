#include "controllers/SimpleController.hpp"


DrivetrainCommand_s SimpleTorqueController::evaluate(const VCRData_s &state, unsigned long curr_millis)
{
    DrivetrainCommand_s out = { .control_mode = _params.control_mode,
                                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}
    };

    const PedalsSystemData_s &pedals_data = state.interface_data.recvd_pedals_data.pedals_data;

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;

    if (_params.control_mode == DrivetrainControlMode_e::TORQUE)
    {
        if (accel_request >= 0.0f)
        {
            // Total torque budget across all 4 motors (motor_max_torque_nm is a single motor's rating, which is why we do *4)
            float total_torque_request = accel_request * _params.motor_max_torque_nm * 4.0f;

            float rear_torque_fraction = _params.accel_bias;
            float front_torque_fraction = 1.0f - rear_torque_fraction;

            float rears_torque_share = total_torque_request * rear_torque_fraction;
            float fronts_torque_share = total_torque_request * front_torque_fraction;

            out.desired_torques.FL = fronts_torque_share / 2.0f;
            out.desired_torques.FR = fronts_torque_share / 2.0f;
            out.desired_torques.RL = rears_torque_share / 2.0f;
            out.desired_torques.RR = rears_torque_share / 2.0f;
        }
        else
        {
            /**
             * @note accel_request is negative here, motor_max_regen_torque_nm is positive, so total_torque_request is negative
             *       sign signals regen/braking, matching how InverterInterface routes negative torque to SET_BRAKE_CURRENT
            */
            float total_torque_request = _params.motor_max_regen_torque_nm * accel_request;

            // regen_bias IS the rear axle's fraction of total regen torque (same convention as accel_bias)
            float rear_torque_fraction = _params.regen_bias;
            float front_torque_fraction = 1.0f - rear_torque_fraction;

            float rears_torque_share = total_torque_request * rear_torque_fraction;
            float fronts_torque_share = total_torque_request * front_torque_fraction;

            out.desired_torques.FL = fronts_torque_share / 2.0f;
            out.desired_torques.FR = fronts_torque_share / 2.0f;
            out.desired_torques.RL = rears_torque_share / 2.0f;
            out.desired_torques.RR = rears_torque_share / 2.0f;
        }
    }
    else   // SPEED
    {
        // Same bias/split logic as TORQUE mode, just producing a speed target instead of a torque target.
        if (accel_request >= 0.0f)
        {
            float total_speed_request = accel_request * _params.motor_max_rpm;

            float rear_speed_fraction = _params.accel_bias;
            float front_speed_fraction = 1.0f - rear_speed_fraction;

            float rears_speed_share = total_speed_request * rear_speed_fraction;
            float fronts_speed_share = total_speed_request * front_speed_fraction;

            out.desired_speeds.FL = std::min(0.0f, fronts_speed_share / 2.0f);
            out.desired_speeds.FR = std::min(0.0f, fronts_speed_share / 2.0f);
            out.desired_speeds.RL = std::min(0.0f, rears_speed_share / 2.0f);
            out.desired_speeds.RR = std::min(0.0f, rears_speed_share / 2.0f);
        }
        else
        {
            /**
             * @note SET_ERPM's sign represents spin direction, not regen/braking
             *       Reverse rotation is what we want I beleive
            */
            float total_speed_request = accel_request * _params.motor_max_rpm;

            float rear_speed_fraction = _params.regen_bias;
            float front_speed_fraction = 1.0f - rear_speed_fraction;

            float rears_speed_share = total_speed_request * rear_speed_fraction;
            float fronts_speed_share = total_speed_request * front_speed_fraction;

            out.desired_speeds.FL = std::max(0.0f, fronts_speed_share / 2.0f);
            out.desired_speeds.FR = std::max(0.0f, fronts_speed_share / 2.0f);
            out.desired_speeds.RL = std::max(0.0f, rears_speed_share / 2.0f);
            out.desired_speeds.RR = std::max(0.0f, rears_speed_share / 2.0f);
        }
    }

    return out;
}