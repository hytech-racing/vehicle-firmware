#ifndef LAUNCH_CONTROLLER_H
#define LAUNCH_CONTROLLER_H

/* Standard Library */
#include <stdlib.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <algorithm>
#include <math.h>

/* Local Controller Includes */
#include "PhysicalParameters.h"


/**
 * @brief Launch behavior states. Each waits on a pedal-input threshold before progressing to the next state.
 *
 * @param LAUNCH_NOT_READY commands zero speed target, applies requested regen/brake
 * torque, and confirms pedals aren't pressed and the car is stationary. Only
 * ever progresses forward to LAUNCH_READY.
 *
 * LAUNCH_READY: commands zero speed target, confirms brake isn't pressed past
 * threshold (0.2 / 20%) and the car stays below the speed threshold (5 m/s).
 * Progresses forward to LAUNCHING once the accelerator is pressed enough, or
 * back to LAUNCH_NOT_READY if either condition is lost.
 *
 * LAUNCHING: ramps commanded torque toward the target, aiming to reach the
 * maximum available torque. Exits back to LAUNCH_NOT_READY if the brake is
 * pressed past threshold or the accelerator drops to or below 0.5 (50%).
 */
enum class LaunchStates_e
{
    NO_LAUNCH_MODE,
    LAUNCH_NOT_READY,
    LAUNCH_READY,
    LAUNCHING
};

namespace launch_controller_default_params
{
    constexpr float DEFAULT_INIT_LAUNCH_TORQUE_NM = 5.0f;
    constexpr float DEFAULT_LAUNCH_TORQUE_RAMP_RATE_NM_PER_SEC = 40.0f;
    constexpr float DEFAULT_MAX_WHEELSPIN_RPM_DELTA = 300.0f;

    constexpr float LAUNCH_READY_ACCEL_THRESH = 0.1f;   // Max pedal is 1.0 I beleive, might be -1.0 to 1.0?
    constexpr float LAUNCH_READY_BRAKE_THRESH = 0.2f;
    constexpr float LAUNCH_READY_SPEED_THRESH_RPM = 5.0f * physical_motor_scales::METERS_PER_SECOND_TO_RPM;

    constexpr float LAUNCH_GO_ACCEL_THRESH = 0.9f;
    constexpr float LAUNCH_STOP_ACCEL_THRESH = 0.5f;
}

struct LaunchControllerThresholds_s
{
    float launch_ready_accel_thresh;
    float launch_ready_brake_thresh;
    float launch_ready_speed_thresh_rpm;
    float launch_go_accel_thresh;
    float launch_stop_accel_thresh;
};

struct LaunchControllerParams_s
{
    float default_init_launch_torque_nm;
    float default_launch_torque_ramp_rate_nm_per_sec;
    float default_max_wheelspin_rpm_delta;
    LaunchControllerThresholds_s thresholds;
};

class SimpleLaunchController
{
public:

    explicit SimpleLaunchController(LaunchControllerParams_s params)
        : _params(params)
    {};

    /**
     * @brief Default Constructor
    */
    SimpleLaunchController()
        : _params {
            .default_init_launch_torque_nm = launch_controller_default_params::DEFAULT_INIT_LAUNCH_TORQUE_NM,
            .default_launch_torque_ramp_rate_nm_per_sec = launch_controller_default_params::DEFAULT_LAUNCH_TORQUE_RAMP_RATE_NM_PER_SEC,
            .default_max_wheelspin_rpm_delta = launch_controller_default_params::DEFAULT_MAX_WHEELSPIN_RPM_DELTA,
            .thresholds = {
                .launch_ready_accel_thresh = launch_controller_default_params::LAUNCH_READY_ACCEL_THRESH,
                .launch_ready_brake_thresh = launch_controller_default_params::LAUNCH_READY_BRAKE_THRESH,
                .launch_ready_speed_thresh_rpm = launch_controller_default_params::LAUNCH_READY_SPEED_THRESH_RPM,
                .launch_go_accel_thresh = launch_controller_default_params::LAUNCH_GO_ACCEL_THRESH,
                .launch_stop_accel_thresh = launch_controller_default_params::LAUNCH_STOP_ACCEL_THRESH
            }
        }
    {};

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, uint32_t curr_millis);

    LaunchStates_e get_launch_state() { return _launch_state; }

private:

    LaunchStates_e _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
    uint32_t _time_of_launch = 0;
    LaunchControllerParams_s _params;

    /**
     * @brief Ramps commanded torque over time during LAUNCHING, starting from init_launch_torque_nm and increasing at
     *        launch_torque_ramp_rate_nm_per_sec, capped at the motor's max torque.
    */
    float _calculate_launch_torque(uint32_t curr_millis) const
    {
        float secs_since_launch = (curr_millis - _time_of_launch) / 1000.0f;
        float ramped_torque =
            _params.default_init_launch_torque_nm + secs_since_launch * _params.default_launch_torque_ramp_rate_nm_per_sec;
        return std::min(dti_motor_params::MOTOR_MAX_TORQUE_NM, std::max(0.0f, ramped_torque));
    }
};

#endif // LAUNCH_CONTROLLER_H