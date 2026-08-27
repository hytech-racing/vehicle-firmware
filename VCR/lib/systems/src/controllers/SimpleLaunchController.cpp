#include "controllers/SimpleLaunchController.h"


DrivetrainCommand_s SimpleLaunchController::evaluate(const VCRData_s &vcr_data, uint32_t curr_millis)
{
    DrivetrainCommand_s out = { .torque_setpoints = {0.0f, 0.0f, 0.0f, 0.0f}};

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;

    int16_t brake_torque_requested_nm = static_cast<int16_t>(pedals_data.brake_percent * dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM);

    // Find the speed (rpm) of the fastest wheel
    float max_wheel_speed_rpm = 0;
    veh_vec<speed_rpm> measured_wheel_speeds = vcr_data.system_data.drivetrain_data.measured_speeds;
    std::array<float, 4> wheel_speeds_array = measured_wheel_speeds.as_array();

    for (int i = 0; i < 4; i++)
    {
        max_wheel_speed_rpm = std::max(max_wheel_speed_rpm, abs(wheel_speeds_array[i]));
    }

    switch (_launch_state)
    {
        case LaunchStates_e::LAUNCH_NOT_READY:
        {
            out.torque_setpoints.FL = brake_torque_requested_nm;
            out.torque_setpoints.FR = brake_torque_requested_nm;
            out.torque_setpoints.RL = brake_torque_requested_nm;
            out.torque_setpoints.RR = brake_torque_requested_nm;

            // init launch vars
            _launch_speed_target_rpm = 0;
            _time_of_launch = curr_millis;

            if ((pedals_data.accel_percent < _params.thresholds.launch_ready_accel_thresh) &&
                (pedals_data.brake_percent < _params.thresholds.launch_ready_brake_thresh) &&
                (max_wheel_speed_rpm < _params.thresholds.launch_ready_speed_thresh_rpm)
            )
            {
                _launch_state = LaunchStates_e::LAUNCH_READY;
            }

            break;
        }
        case LaunchStates_e::LAUNCH_READY:
        {
            out.torque_setpoints.FL = brake_torque_requested_nm;
            out.torque_setpoints.FR = brake_torque_requested_nm;
            out.torque_setpoints.RL = brake_torque_requested_nm;
            out.torque_setpoints.RR = brake_torque_requested_nm;

            // init launch vars
            _launch_speed_target_rpm = 0;
            _time_of_launch = curr_millis;

            // check speed is 0 and brake not pressed
            if ((pedals_data.brake_percent >= _params.thresholds.launch_ready_brake_thresh) ||
                (max_wheel_speed_rpm >= _params.thresholds.launch_ready_speed_thresh_rpm))
            {
                _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
            }
            else if (pedals_data.accel_percent >= _params.thresholds.launch_go_accel_thresh)
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

            _launch_speed_target_rpm = calculate_launch_algo(curr_millis);

            out.torque_setpoints.FL = dti_motor_params::MOTOR_MAX_TORQUE_NM;
            out.torque_setpoints.FR = dti_motor_params::MOTOR_MAX_TORQUE_NM;
            out.torque_setpoints.RL = dti_motor_params::MOTOR_MAX_TORQUE_NM;
            out.torque_setpoints.RR = dti_motor_params::MOTOR_MAX_TORQUE_NM;

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