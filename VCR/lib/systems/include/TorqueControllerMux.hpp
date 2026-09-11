#ifndef TORQUECONTROLLERMUX
#define TORQUECONTROLLERMUX

/* External Includes */
#include <array>
#include <functional>
#include <unordered_map>
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"

/**
 * @brief Torque Controller (TC) Mux handles these things:
 *
 *   1) Swapping between controller outputs
 *   2) Turning on and off running of controllers
 *   3) Apply safeties and limits to controller outputs
*/

namespace tc_mux_default_params
{
    constexpr float MAX_SPEED_DURING_MODE_CHANGE = 5.0; // m/s
    constexpr torque_nm MAX_TORQUE_DELTA_DURING_MODE_CHANGE = 0.5;
    constexpr float MAX_POWER_LIMIT_WATTS = 60000.0;  // watts of mechanical power
};

struct TCMuxParams_s
{
    float max_speed_during_mode_change;
    float max_torque_delta_during_mode_change;
    float max_power_limit_watts;
    uint8_t num_motors;
};

/**
 * @param num_controllers the number of controllers that can be switched between. Defaults to 5 if using TCMuxType.
*/
template <uint8_t num_controllers> class TorqueControllerMux
{
    static_assert(num_controllers > 0, "Must create TC mux with at least 1 controller");

public:

    TorqueControllerMux() = delete;

    /**
     * @brief Constructor for the TC Mux
     * @param controller_evals the array of controller evaluation functions that are being muxed between
     * @param mux_bypass_limits the array of aligned bools for determining if the limits should be  applied to the controller outputs defaults to TC_MUX_DEFAULT_PARAMS::MAX_SPEED_FOR_MODE_CHANGE
    */
    explicit TorqueControllerMux(std::array<std::function<DrivetrainCommand_s(const VCRData_s &state, unsigned long curr_millis)>, num_controllers> controller_evals,
                                std::array<bool, num_controllers> mux_bypass_limits,
                                TCMuxParams_s params = {
                                    .max_speed_during_mode_change = tc_mux_default_params::MAX_SPEED_DURING_MODE_CHANGE,
                                    .max_power_limit_watts = tc_mux_default_params::MAX_POWER_LIMIT_WATTS,
                                    .max_power_limit_watts = tc_mux_default_params::MAX_POWER_LIMIT_WATTS,
                                    .num_motors = 4;
                                }
    ) : _controller_evals(controller_evals),
        _mux_bypass_limits(mux_bypass_limits),
        _params(params)
    {};

    const TorqueControllerMuxStatus_s &getTCMuxStatus() const { return _active_status; }

    /// @brief function that evaluates the mux, controllers and gets the active command
    /// @param requested_controller_type the requested controller type from the dial state
    /// @param controller_command_torque_limit the torque limit state enum set by dashboard
    /// @param input_state the active state of the car
    /// @return the active DrivetrainCommand_s to be sent to the drivetrain to command increases and
    /// decreases in torque
    DrivetrainCommand_s getDrivetrainCommand(ControllerMode_e requested_controller_type,
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

    TCMuxParams_s _params;
    DrivetrainCommand_s _prev_command = {};
    TorqueControllerMuxStatus_s _active_status = {};

    /**
     * @brief Checks whether it's currently safe to switch to a different torque controller
     *        A switch is only considered safe near a complete stop
     * @note Per driver feedback, we don't hard-block switching unless completely stationary.
     *       Instead we allow it below a speed/torque-delta threshold. This is acceptable for
     *       safety since we have reasonably slow, safe cutoffs
     * @return true if the switch is safe to perform, false otherwise
    */
    bool _canSwitchController(DrivetrainDynamicReport_s active_drivetrain_data,
                            DrivetrainCommand_s previous_controller_command,
                            DrivetrainCommand_s desired_controller_out
    );


    /// @brief Ensure torque is at most at the specified limit. If exceeding, then limit it in the
    /// returned DrivetrainCommand_s
    /// @param const DrivetrainCommand_s &command is a DrivetrainCommand_s, which provides torque
    /// info
    /// @param float max_torque this is the maximum average torque the wheels are allowed to
    /// experience before it is limited.
    /// @return DrivetrainCommand_s to update the drivetrain command in the getDrivetrainCommand
    /// method
    /**
     * @brief 
    */
    DrivetrainCommand_s _applyTorqueLimit(const DrivetrainCommand_s &command, float max_torque);

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