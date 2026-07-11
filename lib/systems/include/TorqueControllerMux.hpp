#ifndef TORQUECONTROLLERMUX
#define TORQUECONTROLLERMUX

/* External Includes */
#include <array>
#include <functional>
#include <unordered_map>
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"

// notes:
// 21 torque limit should be first

// tc mux needs to handle these things:
// 1 swapping between controller outputs
// 2 turning on and off running of controllers
// 3 application of safeties and limits to controller outputs
// 4 torque limit changing (torque mode) -->
// TODO the torque limit value changing should be handled in the dashboard interface

// TODOs
// - [x] make the controllers inherit from the base controller class
//      - [x] port TorqueControllerSimple
//      - [x] port TorqueControllerLoadCellVectoring
//      - [x] port BaseLaunchController
//      - [x] port TorqueControllerSimpleLaunch
//      - [x] port slip launch
//      - [x] port TorqueControllerLookupLaunch
//      - [x] port CASE
// - [x] add the torque limit evaluation logic into dashboard interface
// - [x] integrate into state machine
//   - [x] pass through the car state
//   - [x] get dial and torque mode from the dashboard
// - [x] create car_state in main and pass into state machine
// - [x] add 3 bit status to a telemetry message for the TC mux status to HT_CAN
// - [x] pass state of tc mux into telem interface and add the CAN signal
// - [x] remove the old tc mux
// - [ ] add back checking of the ready flag of the controllers and if the controller isnt ready it
// defaults
//        - [ ] add test for this
// - [x] make folder for the controllers
// - [ ] write integration tests for the real controllers
//      - [x] test construction with real controllers
//      - [ ] ensure that sane outputs occur on first tick of each controller
// - [x] update the state machine unit test with integration test of new tc mux

// ON CAR testing
// - [x] test the change of the torque mode from the dashboard interface
//      - [ ] write testing code for this in separate environment

// - [x] switch to using bound controller evaluation functions instead of using polymorphism

/// @brief Contains a max speed for mode changes(5 m/s), a max torque delta for mode change(.5 nm)
/// and a max power limit(63000 W).
///        These values are used in the event that no value is provided for them in the constructor.
namespace TC_MUX_DEFAULT_PARAMS
{
    constexpr const float MAX_SPEED_FOR_MODE_CHANGE = 5.0;        // m/s
    constexpr const float MAX_TORQUE_DELTA_FOR_MODE_CHANGE = 0.5; // Nm
    constexpr const float MAX_POWER_LIMIT = 60000.0;              // watts of mechanical power
}; // namespace TC_MUX_DEFAULT_PARAMS

/// @brief the torque controller muxer that can handle live switching between controller modes
/// @tparam num_controllers the number of controllers that can be switched between. defaults to 5 if
/// using TCMuxType.
template <std::size_t num_controllers> class TorqueControllerMux
{
    static_assert(num_controllers > 0, "Must create TC mux with at least 1 controller");

public:

    TorqueControllerMux() = delete;

    /// @brief constructor for the TC mux
    /// @param controller_evals the array of controller evaluation functions that are being muxed
    /// between
    /// @param mux_bypass_limits the array of aligned bools for determining if the limits should be
    /// applied to the controller outputs defaults to
    /// TC_MUX_DEFAULT_PARAMS::MAX_SPEED_FOR_MODE_CHANGE
    /// @param max_change_speed the max speed difference between the requested controller output and
    /// the actual speed of each wheel that if the controller has a diff larger than the mux will
    /// not switch to the requested controller
    /// @param max_torque_pos_change_delta same as speed but evaluated between the controller
    /// commanded torques defaults to TC_MUX_DEFAULT_PARAMS::MAX_TORQUE_DELTA_FOR_MODE_CHANGE
    /// @param max_power_limit the max power limit defaults to
    /// TC_MUX_DEFAULT_PARAMS::MAX_POWER_LIMIT
    /// @param num_motors the number of motors. defaults to 4.
    /// @note TC Mux must be created with at least 1 controller.
    explicit TorqueControllerMux(std::array<std::function<DrivetrainCommand_s(const VCRData_s &state, unsigned long curr_millis)>, num_controllers> controller_evals,
                            std::array<bool, num_controllers> mux_bypass_limits,
                            float max_change_speed = TC_MUX_DEFAULT_PARAMS::MAX_SPEED_FOR_MODE_CHANGE,
                            float max_torque_pos_change_delta = TC_MUX_DEFAULT_PARAMS::MAX_TORQUE_DELTA_FOR_MODE_CHANGE,
                            float max_power_limit = TC_MUX_DEFAULT_PARAMS::MAX_POWER_LIMIT, size_t num_motors = 4
    ) : _controller_evals(controller_evals),
        _mux_bypass_limits(mux_bypass_limits),
        _max_change_speed(max_change_speed),
        _max_torque_pos_change_delta(max_torque_pos_change_delta),
        _max_power_limit(max_power_limit),
        _num_motors(num_motors)
    {};

    const TorqueControllerMuxStatus_s &get_tc_mux_status() const { return _active_status; }

    /// @brief function that evaluates the mux, controllers and gets the active command
    /// @param requested_controller_type the requested controller type from the dial state
    /// @param controller_command_torque_limit the torque limit state enum set by dashboard
    /// @param input_state the active state of the car
    /// @return the active DrivetrainCommand_s to be sent to the drivetrain to command increases and
    /// decreases in torque
    DrivetrainCommand_s get_drivetrain_command(ControllerMode_e requested_controller_type,
                                               TorqueLimit_e controller_command_torque_limit,
                                               const VCRData_s &input_state
    );

private:

    std::array<std::function<DrivetrainCommand_s(const VCRData_s &state, unsigned long curr_millis)>, num_controllers> _controller_evals;

    std::array<bool, num_controllers> _mux_bypass_limits;

    std::unordered_map<TorqueLimit_e, float> _torque_limit_map = {
        {TorqueLimit_e::TCMUX_FULL_TORQUE, PhysicalParameters::AMK_MAX_TORQUE},
        {TorqueLimit_e::TCMUX_MID_TORQUE, 15.0f},
        {TorqueLimit_e::TCMUX_LOW_TORQUE, 10.0f}
    };

    float _max_change_speed;
    float _max_torque_pos_change_delta;
    float _max_power_limit;
    size_t _num_motors;
    DrivetrainCommand_s _prev_command = {};
    TorqueControllerMuxStatus_s _active_status = {};
    TorqueControllerMuxError_e
    can_switch_controller(DrivetrainDynamicReport_s active_drivetrain_data,
                          DrivetrainCommand_s previous_controller_command,
                          DrivetrainCommand_s desired_controller_out
    );

    /// @brief Clamps negative rpms to 0f
    /// @param const DrivetrainCommand_s &command provides the rpm info as a DrivetrainCommand_s
    /// @return DrivetrainCommand_s to update the drivetrain command in the getDrivetrainCommand
    /// method
    DrivetrainCommand_s _apply_positive_speed_limit(const DrivetrainCommand_s &command);

    /// @brief Ensure torque is at most at the specified limit. If exceeding, then limit it in the
    /// returned DrivetrainCommand_s
    /// @param const DrivetrainCommand_s &command is a DrivetrainCommand_s, which provides torque
    /// info
    /// @param float max_torque this is the maximum average torque the wheels are allowed to
    /// experience before it is limited.
    /// @return DrivetrainCommand_s to update the drivetrain command in the getDrivetrainCommand
    /// method
    DrivetrainCommand_s _apply_torque_limit(const DrivetrainCommand_s &command, float max_torque);

    /// @brief Apply power limit (watts) such that the mechanical power of all wheels never exceeds
    /// the preset mechanical power limit. Scales all wheels down to preserve functionality of
    /// torque controllers
    /// @param const DrivetrainCommand_s &command provides torque info, which is used to calculate
    /// mechanical power
    /// @param const DrivetrainDynamicReport_s &drivetrain provides RPMS, which are used to
    /// calculate radians / s
    /// @param float max_torque is used to indirectly specifiy the max power
    /// @return DrivetrainCommand_s to update the drivetrain command in the getDrivetrainCommand
    /// method
    DrivetrainCommand_s _apply_power_limit(const DrivetrainCommand_s &command,
                                          const DrivetrainDynamicReport_s &drivetrain,
                                          float power_limit, float max_torque
    );

    /// @brief begin limiting regen at noRegenLimitKPH (hardcoded in func) and completely limit
    /// regen at fullRegenLimitKPH (hardcoded in func)
    /// @param const DrivetrainCommand_s &command
    /// @param const DrivetrainDynamicReport_s &drivetrain_data provides RPMs
    /// @return DrivetrainCommand_s to update the drivetrain command in the getDrivetrainCommand
    /// method
    DrivetrainCommand_s _apply_regen_limit(const DrivetrainCommand_s &command,
                                          const DrivetrainDynamicReport_s &drivetrain_data,
                                          const ACUCoreData_s acu_data
    );

};

const int number_of_controllers = 5;
using TCMuxType = TorqueControllerMux<number_of_controllers>;

const int number_of_controllers_min_viable = 1;
using TCMuxTypeMinViable = TorqueControllerMux<number_of_controllers_min_viable>;

#include "TorqueControllerMux.tpp"
#endif // __TorqueControllerMux_H__