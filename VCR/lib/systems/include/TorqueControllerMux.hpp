#ifndef TORQUECONTROLLERMUX
#define TORQUECONTROLLERMUX

/* External Includes */
#include <array>
#include <functional>
#include <unordered_map>
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"

class VCRControls; // forward declaration for "friend"

/**
 * @note Torque Controller (TC) Mux handles these things:
 *
 *   1) Swapping between controller outputs
 *   2) Turning on and off running of controllers
 *   3) Apply safeties and limits to controller outputs
*/

namespace tc_mux_default_params
{
    constexpr float MAX_SPEED_DURING_MODE_CHANGE = 5.0f; // m/s
    constexpr torque_nm MAX_TORQUE_DELTA_DURING_MODE_CHANGE = 0.5f;
    constexpr float MAX_POWER_LIMIT_WATTS = 60000.0f;  // watts of mechanical power
    constexpr torque_nm MAX_AVG_TORQUE_NM = 1.0f; // TODO: PLACEHOLDER -- must be set to a real value before use,
    constexpr speed_rpm MAX_AVG_SPEED_RPM = 1.0f; // TODO: PLACEHOLDER -- must be set to a real value before use
};

struct TCMuxParams_s
{
    float max_speed_during_mode_change;
    torque_nm max_torque_delta_during_mode_change;
    float max_power_limit_watts;
    torque_nm max_avg_torque_nm;
    speed_rpm max_avg_speed_rpm;
    uint8_t num_motors;
};

/**
 * @note We will choose not to use singleton structure here and force TC mux creation to be controlled by VCRControls which is the
 *      "highest" layer of control overseeing TC Mux. We will prohibit direct access to the mux.
 * @param num_controllers the number of controllers that can be switched between. Defaults to 5 if using TCMuxType.
*/
template <size_t num_controllers> class TorqueControllerMux
{
    static_assert(num_controllers > 0, "Must create TC mux with at least 1 controller");

    /**
     * @note We will choose not use singleton structure and have VCRControls as the highest layer of control overseeing torque decisions
     *       As a safeguard, we will not allow it to be independently constructible: it exists only as VCRControls's private implementation
     *       detail. "friend" + a private constructor make this compiler-enforced
     */
    friend class VCRControls;

public:

    TorqueControllerMux() = delete;

    const TorqueControllerMuxStatus_s &getTCMuxStatus() const { return _curr_tc_mux_status; }

    /**
     * @brief Main entry point of the TC Mux
     * @note Evaluates the active controller for this tick, and if a different mode has been requested,
     *       evaluates that mode's controller too and checks whether it's safe to switch to it right now.
     *       If the switch is approved, it becomes the new active mode. Also, unless the resulting active
     *       mode has limits bypassed, limiting is applied
     * @param requested_controller_type the controller mode requested by the driver/dashboard
     * @param input_state the current state of the car
     * @return the DrivetrainCommand_s to send to the drivetrain this tick. Returns an
     *         EMPTY_COMMAND (zero torque/speed) if requested_controller_type is unsupported,
     *         or if either the active or requested controller's slot in controller_evals is
     *         unbound
     *
     * TODO: Consider changing name, evaluateTCMux might be confusing
    */
    DrivetrainCommand_s evaluateTCMux(ControllerMode_e requested_controller_type,
                                    const VCRData_s &input_state
    );

private:

    /**
     * @brief Private constructor for the TC Mux
     * @param controller_evals an array of size num_controllers, holds the various controllers "evaluate" methods
     * @param mux_bypass_limits an array of size num_controllers, holds bools for determining if the limit methods should be
     *                          applied to the controller outputs
    */
    explicit TorqueControllerMux(
                std::array<std::function<DrivetrainCommand_s(const VCRData_s &state, unsigned long curr_millis)>, num_controllers> controller_evals,
                std::array<bool, num_controllers> mux_bypass_limits,
                TCMuxParams_s params = {
                    .max_speed_during_mode_change = tc_mux_default_params::MAX_SPEED_DURING_MODE_CHANGE,
                    .max_torque_delta_during_mode_change = tc_mux_default_params::MAX_TORQUE_DELTA_DURING_MODE_CHANGE,
                    .max_power_limit_watts = tc_mux_default_params::MAX_POWER_LIMIT_WATTS,
                    .max_avg_torque_nm = tc_mux_default_params::MAX_AVG_TORQUE_NM,
                    .max_avg_speed_rpm = tc_mux_default_params::MAX_AVG_SPEED_RPM,
                    .num_motors = 4
                }
    ) : _controller_evals(controller_evals),
        _mux_bypass_limits(mux_bypass_limits),
        _params(params)
    {};

    std::array<std::function<DrivetrainCommand_s(const VCRData_s &state, unsigned long curr_millis)>, num_controllers> _controller_evals;
    std::array<bool, num_controllers> _mux_bypass_limits;
    TCMuxParams_s _params;
    DrivetrainCommand_s _prev_command = {};
    TorqueControllerMuxStatus_s _curr_tc_mux_status = {};

    /**
     * @brief Checks whether it's currently safe to switch to a different torque controller
     * @note A switch is only considered safe NEAR a complete stop. Per driver feedback, we don't force stationary, but use a
     *       speed/torque-delta threshold. This is acceptable for safety since we have reasonably slow, safe cutoffs
     * @param active_mode_evaluate_output is the DrivetrainCommand_s which is returned/evaulated by current controller/mode
     * @param requested_mode_evaluate_output is the DrivetrainCommand_s which is returned/evaulated by requested controller/mode
     * @return true if the switch is safe to perform, false otherwise
    */
    bool _canSwitchController(DrivetrainDynamicReport_s active_drivetrain_data,
                            DrivetrainCommand_s active_mode_evaluate_output,
                            DrivetrainCommand_s requested_mode_evaluate_output
    );

    /**
     * @brief Apply torque limit such that the average torque requested cannot exceed the limit. If exceeding,
     *        then scale it down accordingly (preserve torque ratios)
     * @param desired_controller_out is a DrivetrainCommand_s requesting some torque
     * @param max_avg_torque_limit is maximum average torque the wheels are allowed to experience
     * @return DrivetrainCommand_s to update the drivetrain command in evaluateTCMux
    */
    DrivetrainCommand_s _applyTorqueLimit(const DrivetrainCommand_s &desired_controller_out, float max_avg_torque_limit);

    /**
     * @brief Works the same as _applyTorqueLimit, just applied to desired_speeds instead of desired_torques,
     *        for when we are controlling motors using speed instead of torque
     * @param desired_controller_out is a DrivetrainCommand_s requesting some speed
     * @param max_avg_speed_limit is maximum average speed the wheels are allowed to be commanded to
     * @return DrivetrainCommand_s to update the drivetrain command in evaluateTCMux
    */
    DrivetrainCommand_s _applySpeedLimit(const DrivetrainCommand_s &desired_controller_out, float max_avg_speed_limit);

    /**
     * @brief Apply power limit (watts) such that the mechanical power of all wheels never exceeds the preset mechanical power limit.
     *        If exceeding, then scale it down accordingly (preserve torque ratios)
     * @param desired_controller_out is a DrivetrainCommand_s requesting some torque
     * @param dynamic_report is a DrivetrainDynamicReport_s which provides the active RPMs
     * @param power_limit_watts is the mechanical power limit (watts)
     * @param max_torque_nm hard per-wheel torque ceiling the scaled-down output is clamped to
     * @return DrivetrainCommand_s to update the drivetrain command in evaluateTCMux
    */
    DrivetrainCommand_s _applyTorqueControlPowerLimit(const DrivetrainCommand_s &desired_controller_out,
                                                    const DrivetrainDynamicReport_s &dynamic_report,
                                                    float power_limit_watts,
                                                    float max_torque_nm
    );

    /**
     * @brief Works the same as _applyTorqueControlPowerLimit, just used when we are controlling motors using speed instead of torque
     * @param desired_controller_out is a DrivetrainCommand_s requesting some speed
     * @param dynamic_report is a DrivetrainDynamicReport_s which provides the active measured torques/speeds
     * @param power_limit_watts is the mechanical power limit (watts)
     * @param max_speed_rpm hard per-wheel speed ceiling the scaled-down output is clamped to
     * @warning This is an approximation, not exact like the torque-mode correction. It assumes measured torque stays roughly constant as the speed target changes
     *          which isn't true since DTI's internal speed loop will re-derive torque at the new target
    */
    DrivetrainCommand_s _applySpeedControlPowerLimit(const DrivetrainCommand_s &desired_controller_out,
                                                    const DrivetrainDynamicReport_s &dynamic_report,
                                                    float power_limit_watts,
                                                    float max_speed_rpm
    );

    /**
     * @brief Apply regen limits based on EV.3.3.3
     * @note Speed vs Torque control handling is done inside the method
     * @param desired_controller_out is a DrivetrainCommand_s requesting some torque/speed
     * @param dynamic_report is a DrivetrainDynamicReport_s which provides RPMs
     * @param acu_data is used for electrical power, then power limit? NOTE: currently not used
     * @return DrivetrainCommand_s to update the drivetrain command in evaluateTCMux
    */
    DrivetrainCommand_s _applyRegenLimit(const DrivetrainCommand_s &desired_controller_out,
                                        const DrivetrainDynamicReport_s &dynamic_report,
                                        const ACUCoreData_s acu_data
    );

};

const int number_of_controllers = 5;
using TCMuxType = TorqueControllerMux<number_of_controllers>;

const int number_of_controllers_min_viable = 1;
using TCMuxTypeMinViable = TorqueControllerMux<number_of_controllers_min_viable>;

#include "TorqueControllerMux.tpp"
#endif // TORQUECONTROLLERMUX