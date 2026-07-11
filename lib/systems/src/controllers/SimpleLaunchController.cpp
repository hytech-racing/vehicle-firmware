#include "controllers/SimpleLaunchController.h"


DrivetrainCommand_s SimpleLaunchController::evaluate(const VCRData_s &vcr_data, uint32_t curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f}
    };

    const PedalsSystemData_s &pedalsData = vcr_data.interface_data.recvd_pedals_data.pedals_data;

    int16_t brake_torque_req = static_cast<int16_t>(pedalsData.brake_percent * PhysicalParameters::MAX_REGEN_TORQUE);

    float max_speed = 0;
    veh_vec<speed_rpm> measured_speeds = vcr_data.system_data.drivetrain_data.measuredSpeeds;
    std::array<float, 4> wheel_rpms = measured_speeds.as_array();
    for (int i = 0; i < 4; i++)
    {
        max_speed = std::max(max_speed, abs(wheel_rpms[i]));
    }

    switch (_launch_state)
    {
        case LaunchStates_e::LAUNCH_NOT_READY:
        {
            out.torque_limits.FL = brake_torque_req;
            out.torque_limits.FR = brake_torque_req;
            out.torque_limits.RL = brake_torque_req;
            out.torque_limits.RR = brake_torque_req;

            // init launch vars
            _launch_speed_target_rpm = 0;
            _time_of_launch = curr_millis;

            // check speed is 0 and pedals not pressed
            if ((pedalsData.accel_percent < LaunchControllerParams::launch_ready_accel_threshold) && (pedalsData.brake_percent < LaunchControllerParams::launch_ready_brake_threshold) && (max_speed < LaunchControllerParams::launch_ready_speed_threshold))
            {
                _launch_state = LaunchStates_e::LAUNCH_READY;
            }

            break;
        }
        case LaunchStates_e::LAUNCH_READY:
        {
            out.torque_limits.FL = brake_torque_req;
            out.torque_limits.FR = brake_torque_req;
            out.torque_limits.RL = brake_torque_req;
            out.torque_limits.RR = brake_torque_req;

            // init launch vars
            _launch_speed_target_rpm = 0;
            _time_of_launch = curr_millis;

            // check speed is 0 and brake not pressed
            if ((pedalsData.brake_percent >= LaunchControllerParams::launch_ready_brake_threshold) || (max_speed >= LaunchControllerParams::launch_ready_speed_threshold))
            {
                _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
            }
            else if (pedalsData.accel_percent >= LaunchControllerParams::launch_go_accel_threshold)
            {
                _launch_state = LaunchStates_e::LAUNCHING;
            }

            break;
        }
        case LaunchStates_e::LAUNCHING:
        {
            // use brackets to ignore 'cross initialization' of secs_since_launch
            // check accel below launch threshold and brake above
            if ((pedalsData.accel_percent <= LaunchControllerParams::launch_stop_accel_threshold) || (pedalsData.brake_percent >= LaunchControllerParams::launch_ready_brake_threshold))
            {
                _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
            }

            _launch_speed_target_rpm = calc_launch_algo(curr_millis);

            out.torque_limits.FL = PhysicalParameters::AMK_MAX_TORQUE;
            out.torque_limits.FR = PhysicalParameters::AMK_MAX_TORQUE;
            out.torque_limits.RL = PhysicalParameters::AMK_MAX_TORQUE;
            out.torque_limits.RR = PhysicalParameters::AMK_MAX_TORQUE;

            break;
        }
        default:
        {
            break;
        }
    }

    out.desired_speeds.FL = _launch_speed_target_rpm;
    out.desired_speeds.FR = _launch_speed_target_rpm;
    out.desired_speeds.RL = _launch_speed_target_rpm;
    out.desired_speeds.RR = _launch_speed_target_rpm;

    return out;
}