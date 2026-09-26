#include "controllers/SimpleLaunchController.hpp"


DrivetrainCommand_s SimpleLaunchController::evaluate(const VCRData_s &vcr_data, uint32_t curr_millis)
{
    DrivetrainCommand_s out = { .control_mode = DrivetrainControlMode_e::TORQUE,
                                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}
    };

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;

    // Negative sign matches the convention InverterInterface uses to route to SET_BRAKE_CURRENT.
    float brake_torque_requested_nm = - (pedals_data.brake_percent * dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM);

    auto wheel_speeds_array = vcr_data.system_data.drivetrain_data.measured_speeds.as_array();
    float max_wheel_speed_rpm = 0.0f;
    for (const auto& wheel_speed : wheel_speeds_array)
    {
        max_wheel_speed_rpm = std::max(max_wheel_speed_rpm, std::fabs(wheel_speed));
    }

    switch (_launch_state)
    {
        case LaunchStates_e::LAUNCH_NOT_READY:
        {
            out.desired_torques = {
                brake_torque_requested_nm,
                brake_torque_requested_nm,
                brake_torque_requested_nm,
                brake_torque_requested_nm
            };

            bool is_ready_to_arm =
                (pedals_data.accel_percent < _params.thresholds.launch_ready_accel_thresh) &&
                (pedals_data.brake_percent < _params.thresholds.launch_ready_brake_thresh) &&
                (max_wheel_speed_rpm < _params.thresholds.launch_ready_speed_thresh_rpm);

            if (is_ready_to_arm)
            {
                _launch_state = LaunchStates_e::LAUNCH_READY;
            }

            break;
        }
        case LaunchStates_e::LAUNCH_READY:
        {
            out.desired_torques = {
                brake_torque_requested_nm,
                brake_torque_requested_nm,
                brake_torque_requested_nm,
                brake_torque_requested_nm
            };

            bool have_lost_ready_conditions =
                (pedals_data.brake_percent >= _params.thresholds.launch_ready_brake_thresh) ||
                (max_wheel_speed_rpm >= _params.thresholds.launch_ready_speed_thresh_rpm);

            if (have_lost_ready_conditions)
            {
                _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
            }
            else if (pedals_data.accel_percent >= _params.thresholds.launch_go_accel_thresh)
            {
                _time_of_launch = curr_millis;   // reset exactly once, on entry to LAUNCHING
                _launch_state = LaunchStates_e::LAUNCHING;
            }

            break;
        }
        case LaunchStates_e::LAUNCHING:
        {
            bool should_abort_launch =
                (pedals_data.accel_percent <= _params.thresholds.launch_stop_accel_thresh) ||
                (pedals_data.brake_percent >= _params.thresholds.launch_ready_brake_thresh);

            if (should_abort_launch)
            {
                _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
                break;
            }

            float launch_torque_nm = _calculate_launch_torque(curr_millis);
            out.desired_torques = {launch_torque_nm, launch_torque_nm, launch_torque_nm, launch_torque_nm};

            break;
        }
        default:
        {
            break;
        }
    }

    return out;
}