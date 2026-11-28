#include "controllers/SimpleLaunchController.hpp"


DrivetrainCommand_s SimpleLaunchController::evaluate(const VCRData_s& vcr_data, uint32_t curr_millis)
{
    DrivetrainCommand_s out = {{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}};

    const PedalsSystemData_s& pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;

    int16_t brake_torque_req = static_cast<int16_t>(pedals_data.brake_percent * PhysicalParameters::MAX_REGEN_TORQUE);

    float max_speed = 0;
    veh_vec<SpeedRpm> measured_speeds = vcr_data.system_data.drivetrain_data.measuredSpeeds;
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
        if ((pedals_data.accel_percent < LaunchControllerParams::LAUNCH_READY_ACCEL_THRESHOLD)
            && (pedals_data.brake_percent < LaunchControllerParams::LAUNCH_READY_BRAKE_THRESHOLD)
            && (max_speed < LaunchControllerParams::LAUNCH_READY_SPEED_THRESHOLD))
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
        if ((pedals_data.brake_percent >= LaunchControllerParams::LAUNCH_READY_BRAKE_THRESHOLD)
            || (max_speed >= LaunchControllerParams::LAUNCH_READY_SPEED_THRESHOLD))
        {
            _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
        }
        else if (pedals_data.accel_percent >= LaunchControllerParams::LAUNCH_GO_ACCEL_THRESHOLD)
        {
            _launch_state = LaunchStates_e::LAUNCHING;
        }

        break;
    }
    case LaunchStates_e::LAUNCHING:
    {
        // use brackets to ignore 'cross initialization' of secs_since_launch
        // check accel below launch threshold and brake above
        if ((pedals_data.accel_percent <= LaunchControllerParams::LAUNCH_STOP_ACCEL_THRESHOLD)
            || (pedals_data.brake_percent >= LaunchControllerParams::LAUNCH_READY_BRAKE_THRESHOLD))
        {
            _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
        }

        _launch_speed_target_rpm = _calcLaunchAlgo(curr_millis);

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
