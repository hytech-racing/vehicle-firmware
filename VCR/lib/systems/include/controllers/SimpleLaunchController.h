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

/// @brief Modes to define launch behavior, where each one waits for acceleration request threshold to move to next mode
/// LAUNCH_NOT_READY keeps speed at 0 and makes sure pedals are not pressed, the launch controller begins in this state
/// From this state the launch can only progress forwards to LAUNCH_READY
/// LAUNCH_READY keeps speed at 0, below the speed threshold(5 m/s) and makes sure brake is not pressed harder than the threshold of .2(20% pushed)
/// From this state the launch can progress forwards to LAUNCHING according to the two conditions defined above or backwards to LAUNCH_NOT_READY if those conditions are not met
/// LAUNCHING uses respective algorithm to set speed set point and requests torque from motors to reach it
/// From this state the launch can fully begin and set speed set points above 0.0 m/s and the maximum available torque can be requested from the inverters
/// This launch state can be terminated if the brake is pressed above the threshold(.2(20% pushed)) or if the accelerator is not pressed enough (<= .5(50% pushed))
enum class LaunchStates_e
{
    NO_LAUNCH_MODE,
    LAUNCH_NOT_READY,
    LAUNCH_READY,
    LAUNCHING
};

/// @brief contains constants for tick behavior/progression(_threshold variables used to determine when to move to the next step) and defaults(DEFAULT_launch_speed_target_rpm)
namespace LaunchControllerParams
{
    const int16_t DEFAULT_INIT_SPEED_RPM = 1500; // Target RPM as soon as simple launch begins
    // const int16_t DEFAULT_INIT_SPEED_RPM = 500; // Target RPM as soon as simple launch begins
    const float DEFAULT_LAUNCH_RATE_M_PER_SEC_SQ = 11.76;
    // const float DEFAULT_LAUNCH_RATE_M_PER_SEC_SQ = 3.0f;
    const float launch_ready_accel_threshold = .1;
    const float launch_ready_brake_threshold = .2;
    const float launch_ready_speed_threshold = 5.0 * METERS_PER_SECOND_TO_RPM; // rpm
    const float launch_go_accel_threshold = .9;
    const float launch_stop_accel_threshold = .5;
}

class SimpleLaunchController
{
public:

    /// @brief simple TC with tunable F/R torque balance. Accel torque balance can be tuned independently of regen torque balance
    SimpleLaunchController() :
        _launch_rate_target_m_per_sec_sq(LaunchControllerParams::DEFAULT_LAUNCH_RATE_M_PER_SEC_SQ),
        _time_of_launch(0),
        _launch_state(LaunchStates_e::LAUNCH_NOT_READY),
        _launch_speed_target_rpm(0),
        _init_speed_target_rpm(LaunchControllerParams::DEFAULT_INIT_SPEED_RPM)
    {};

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, uint32_t curr_millis);

    LaunchStates_e get_launch_state() { return _launch_state; }

private:

    float _launch_rate_target_m_per_sec_sq;
    uint32_t _time_of_launch;
    LaunchStates_e _launch_state = LaunchStates_e::LAUNCH_NOT_READY;
    float _launch_speed_target_rpm;
    int16_t _init_speed_target_rpm;

    /// @brief calculates how speed target (the speed the car is trying to achieve during launch) is set and/or increased during launch
    /// This updates internal speed target variable _launch_speed_target_rpm
    /// @param vn_data vector data needed for calulations eg. speed and coordinates
    /// @note defines important variation in launch controller tick/evaluation as the launch controllers share a tick method defined in this parent class implementation
    /// @note all launch algorithms are implemented in LaunchControllerAlgos.cpp
    float calc_launch_algo(uint32_t curr_millis)
    {
        /*
        Stolen launch algo from HT07. This ramps up the speed target over time.
        launch rate target is m/s^2 and is the target acceleration rate
        secs_since_launch takes the milliseconds since launch started and converts to sec
        This is then converted to RPM for a speed target
        There is an initial speed target that is your iitial instant acceleration on the wheels
        */
        float secs_since_launch = (curr_millis - _time_of_launch) / 1000.0f;
        float calculated_rpm = (int16_t) (secs_since_launch * _launch_rate_target_m_per_sec_sq * METERS_PER_SECOND_TO_RPM);
        calculated_rpm += _init_speed_target_rpm;
        calculated_rpm = std::min( (int) PhysicalParameters::AMK_MAX_RPM, std::max(0, (int) calculated_rpm));
        // calculated_rpm = std::min( (int) 5000, std::max(0, (int) calculated_rpm));
        return calculated_rpm;
    }

};

#endif // LAUNCH_CONTROLLER_H