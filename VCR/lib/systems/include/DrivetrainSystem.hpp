#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include <cmath>


/// NOTE: More structs and data is defined in SharedFirmwareTypes

namespace drivetrain_default_params
{
    constexpr uint16_t CONTROL_MODE_MISMATCH_THRESHOLD_MS = 100;
    constexpr uint16_t DRIVE_ENABLE_TIMEOUT_MS = 500;   // TODO: tune against measured DTI drive-enable response time
}

/**
 * @brief When user calls evaluate_drivetrain(), this is part of the returned status.
 * @note Enum indicates whether the most recent command was actually successful or invalid given the drivetrain's current state.
 *
 * @param COMMAND_OK Command was valid and has been applied
 * @param CANNOT_SEND_NOT_CONNECTED Command was rejected because inverters are not yet connected
 * @param COMMAND_INVALID Command contained NaN/inf or an unknown control mode (motors are idled), or a nonzero command arrived
 *                        while the drivetrain was not READY
*/
enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,
    CANNOT_SEND_NOT_CONNECTED = 1,
    COMMAND_INVALID = 2
};

/**
 * @brief Struct that gets returned on drivetrain evaluations. This is how users can determine
 *        what the drivetrain system is doing and whether the last command was accepted/successful.
 *
 * @param are_all_inverters_connected true only if all four inverters currently report `connected`
 * @param are_all_inverters_enabled true only if all four inverters have confirmed drive-enable
 * @param is_control_mode_mismatch_detected true if, while READY, any inverter's reported control mode has disagreed
 *                                          with what the controller last commanded for longer than the threshold
 * @param inverter_statuses per-corner status detail (FL/FR/RL/RR), so callers can diagnose corner individually
 * @param cmd_response Whether the command just passed into evaluate_drivetrain() was successful or invalid
 * @param current_dtsm_state the drivetrain state machine's state as of this evaluation.
*/
struct DrivetrainStatus_s
{
    bool are_all_inverters_connected;
    bool are_all_inverters_enabled;
    bool is_control_mode_mismatch_detected;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_response;
    DrivetrainState_e current_dtsm_state;
};

/**
 * @brief This struct bundles various hardware-agnostic callbacks DrivetrainSystem uses to talk to the inverters.
 * @note We are just bundling the Inverter API methods as delegates since the DrivetrainSystem should never "see"
 *       InverterInterface directly.
 */
struct InverterInterfaceFuncts_s
{
    etl::delegate<void(torque_nm)> setMotorsTorque;
    etl::delegate<void(speed_rpm)> setMotorsSpeed;
    etl::delegate<void()> setMotorsIdle;
    etl::delegate<void(bool)> requestEnable;
    etl::delegate<bool(DrivetrainControlMode_e)> isReportedModeMatchingDT;
    etl::delegate<InverterStatus_s(unsigned long)> getInverterStatus;
    etl::delegate<MotorMechanics_s()> getMotorMechanics;
    etl::delegate<void(DrivetrainControlMode_e)> sendCommands;
};

struct DrivetrainParams_s
{
    uint16_t control_mode_mismatch_threshold_ms;
    uint16_t drive_enable_timeout_ms;
};

/**
 * @brief The Drivetrain System is primarily responsible for two things:
 *
 *  - Updating its internal state machine
 *  - Determining what commands to give each InverterInterface
 *
 * State Ladder:
 *   NOT_CONNECTED -> CONNECTED_HV_ABSENT -> CONNECTED_HV_PRESENT -> WANTING_OK -> READY
 *   WANTING_OK means drive enable has been requested from the inverters and we are waiting for all four to confirm
 *   FAULTED is entered on any inverter fault code and exits back to NOT_CONNECTED once faults clear
*/
class DrivetrainSystem
{
public:

    DrivetrainSystem() = delete;

    DrivetrainSystem(veh_vec<InverterInterfaceFuncts_s> inverter_interfaces_functs,
                    etl::delegate<bool()> isHVStatusOK,
                    DrivetrainParams_s drivetrain_params = {
                        .control_mode_mismatch_threshold_ms = drivetrain_default_params::CONTROL_MODE_MISMATCH_THRESHOLD_MS,
                        .drive_enable_timeout_ms = drivetrain_default_params::DRIVE_ENABLE_TIMEOUT_MS,
                    }
    ) :
        _drivetrain_params(drivetrain_params),
        _current_state(DrivetrainState_e::NOT_CONNECTED),
        _inverter_interfaces_functs(inverter_interfaces_functs),
        _isHVStatusOK(isHVStatusOK)
    {}

    /**
     * @brief Advances the drivetrain state machine, applies this tick's command if READY, and sends commands to
     *        every inverter (sends commands in every state, so drive-disable and zero-current frames keep going out too)
     * @param command this tick's desired torque or speed plus the drive enable request, built by whichever
     *                controller is currently active. Torques/speeds are only applied if the drivetrain is READY
     * @return a DrivetrainStatus_s snapshot
    */
    DrivetrainStatus_s evaluateDrivetrain(const DrivetrainCommand_s& command, unsigned long current_millis);

    /* GETTERS */
    DrivetrainState_e getCurrentState() const;
    DrivetrainStatus_s getStatus() const;
    const char* getStateName() const;


private:

    DrivetrainParams_s _drivetrain_params;
    DrivetrainState_e _current_state;
    unsigned long _last_state_changed_time = 0;
    DrivetrainStatus_s _status = {};
    veh_vec<InverterInterfaceFuncts_s> _inverter_interfaces_functs;
    veh_vec<InverterStatus_s> _inverter_statuses = {};

    DrivetrainControlMode_e _last_commanded_control_mode = DrivetrainControlMode_e::TORQUE;
    unsigned long _last_control_mode_change_millis = 0;

    /**
     * @brief Method will check DC bus voltage (input voltage, x4) reported by the inverters is within expected thresholds
     *        and does not deviate from what ACU reports is pack voltage
    */
    etl::delegate<bool()> _isHVStatusOK;

    /* INTERNAL FUNCTIONS FOR DTSM */

    /**
     * @brief Method is how we transition states
     * @note Runs exit logic for the current state, updates _current_state, runs entry logic for new_state,
     *       and records the transition time (_last_state_changed_time)
    */
    void _setState(DrivetrainState_e new_state, unsigned long current_millis);

    /**
     * @brief Runs once, on leaving prev_state, before the new state is entered
    */
    void _handleExitLogic(DrivetrainState_e prev_state, unsigned long current_millis);

    /**
     * @brief Runs once, on entering new_state
    */
    void _handleEntryLogic(DrivetrainState_e new_state, unsigned long current_millis);

    /**
     * @brief Method evaluates transition conditions for the current state and calls _set_state if a transition is warranted
     * @note Method only contains pure state-machine logic, does not read or apply a DrivetrainCommand_s (i.e.) does not
     *       command the inverters/motors
     * @return the state after evaluation
    */
    DrivetrainState_e _evaluateStateMachine(const DrivetrainCommand_s& command, unsigned long current_millis);

    /**
     * @brief Fills _inverter_statuses from each inverter's getInverterStatus()
    */
    void _refreshInverterStatuses(unsigned long current_millis);

    /**
     * @brief Method checks if we are connected with each of the inverters.
     *        Connection in this case is defined as we are continously receiving CAN messages.
     * @return true if all inverters are connected, false otherwise
    */
    bool _areAllInvertersConnected() const;

    /**
     * @brief Method checks if any of the 4 inverters (FL, FR, RL, RR) have a fault code
     * @return true if any inverter has a fault code, false otherwise
    */
    bool _isAnyInverterFaulted() const;

    /**
     * @brief Method will check the drive enable flag per motor/inverter
     * @return True if all inverters are in the drive_enabled state, false otherwise
    */
    bool _isDriveEnabled() const;

    /**
     * @brief Method will check the current Drivetrain mode is the same as the current mode reported by the inverters
     * @return True if they are mismatched (after the threshold), false otherwise
    */
    bool _isControlModeMismatched(unsigned long current_millis) const;

    /**
     * @return true if the command's control mode is TORQUE or SPEED and all four values for that mode are finite
    */
    bool _isCommandValid(const DrivetrainCommand_s& command) const;

    /**
     * @return true if any of the four values for the command's control mode is nonzero
    */
    bool _isCommandNonZero(const DrivetrainCommand_s& command) const;

    /**
     * @brief Method will set the drive enable flag per motor/inverter
     *        As redundancy, if the drive enable flag is going to be set low, we will also set the motor idle
     *        This should happen automatically, but since it is untested, we will just add this redundancy feature now
     * @param state is the state the flag will be set to
    */
    void _setDriveEnable(bool state);

    /**
     * @brief Applies this tick's idle command to each corner by calling setMotorsIdle for every inverter
    */
    void _setMotorsIdle();

    /**
     * @brief Applies this tick's torque command to each corner by calling setMotorsTorque for every inverter
    */
    void _setMotorsTorque(const DrivetrainCommand_s& command, unsigned long current_millis);

    /**
     * @brief Applies this tick's speed command to each corner by calling setMotorsSpeed for every inverter
    */
    void _setMotorsSpeed(const DrivetrainCommand_s& command, unsigned long current_millis);

    /**
     * @brief Calls sendCommands on every inverter with the last commanded control mode
    */
    void _sendInverterCommands();

};

using DrivetrainInstance = etl::singleton<DrivetrainSystem>;

#endif /* DRIVETRAINSYSTEM */