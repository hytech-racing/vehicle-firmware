#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"


/**
 * @brief When user calls evaluate_drivetrain(), this is part of the returned status.
 * @note Enum indicates whether the most recent command was actually successful or invalid given the drivetrain's current state.
 *
 * @param COMMAND_OK Command was valid and has been applied.
 * @param CANNOT_SEND_NOT_CONNECTED Command was rejected because inverters are not yet connected.
 * @param COMMAND_INVALID  Reserved for other invalid-command cases
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
 * @param is_control_mode_mismatch_detected true if any inverter's reported control mode disagrees with what the controller last commanded
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
    etl::delegate<void(float torque_nm)> set_motors_torque;
    etl::delegate<void(float speed_rpm)> set_motors_speed;
    etl::delegate<void()> set_motors_idle;
    etl::delegate<void(bool)> request_enable;
    etl::delegate<bool(DrivetrainControlMode_e)> is_reported_mode_matching_dt;
    etl::delegate<InverterStatus_s()> get_status;
    etl::delegate<MotorMechanics_s()> get_motor_mechanics;
};

/**
 * @brief The Drivetrain System is primarily responsible for two things:
 *
 *  - Updating its internal state machine
 *  - Determining what commands to give each InverterInterface
*/
class DrivetrainSystem
{
public:

    DrivetrainSystem() = delete;

    DrivetrainSystem(uint16_t control_mode_mismatch_threshold_ms,
                    veh_vec<InverterInterfaceFuncts_s> inverter_interfaces_functs,
                    etl::delegate<bool()> is_hv_status_ok
    ) :
        _control_mode_mismatch_threshold_ms(control_mode_mismatch_threshold_ms),
        _current_state(DrivetrainState_e::NOT_CONNECTED),
        _inverter_interfaces_functs(inverter_interfaces_functs),
        _is_hv_status_ok(is_hv_status_ok)
    {}

    /**
     * @brief Advances the drivetrain state machine and, if currently in DRIVE_ENABLED,
     *        dispatches this tick's command (torque or speed) to each inverter
     * @param command this tick's desired torque or speed, built by whichever controller is currently active
     *                Only applied if the drivetrain is in DRIVE_ENABLED
     * @param current_millis current system time, used for state-transition timing
     * @return a DrivetrainStatus_s snapshot
    */
    DrivetrainStatus_s evaluate_drivetrain(DrivetrainCommand_s command, unsigned long current_millis);

    /* GETTERS */
    DrivetrainState_e get_current_state() const;
    DrivetrainStatus_s get_status() const;
    const char* get_state_name() const;


private:

    DrivetrainState_e _current_state;
    unsigned long _last_state_changed_time = 0;
    DrivetrainStatus_s _status;
    veh_vec<InverterInterfaceFuncts_s> _inverter_interfaces_functs;

    DrivetrainControlMode_e _last_commanded_control_mode = DrivetrainControlMode_e::TORQUE;
    unsigned long _last_control_mode_change_millis = 0;
    uint16_t _control_mode_mismatch_threshold_ms;

    /**
     * @brief Check if we are latched essentially.
     * @note Checks DC bus voltage (input voltage, x4) reported by the inverters is within expected thresholds.
     *       Will also check that the reported input voltage is not deviating from the reported pack voltage by ACU.
    */
    etl::delegate<bool()> _is_hv_status_ok;


    /* INTERNAL FUNCTIONS FOR DTSM */

    /**
     * @brief Method is how we transition states
     * @note Runs exit logic for the current state, updates _current_state, runs entry logic for new_state,
     * and records the transition time (_last_state_changed_time)
    */
    void _set_state(DrivetrainState_e new_state, unsigned long current_millis);

    /**
     * @brief Runs once, on leaving prev_state, before the new state is entered
    */
    void _handle_exit_logic(DrivetrainState_e prev_state, unsigned long current_millis);

    /**
     * @brief Runs once, on entering new_state
    */
    void _handle_entry_logic(DrivetrainState_e new_state, unsigned long current_millis);

    /**
     * @brief Method evaluates transition conditions for the current state and calls _set_state if a transition is warranted.
     * @note Method only contains pure state-machine logic, does not read or apply a DrivetrainCommand_s (i.e.) does not
     *       command the inverters/motors
     * @return the state after evaluation
    */
    DrivetrainState_e _evaluate_state_machine(unsigned long current_millis);

    /**
     * @brief Method checks if we are connected with each of the inverters.
     *        Connection in this case is defined as we are continously receiving CAN messages.
     * @return true if all inverters are connected, false otherwise
    */
    bool _are_all_inverters_connected();

    /**
     * @brief Method checks if any of the 4 inverters (FL, FR, RL, RR) have a fault code
     * @return true if any inverter has a fault code, false otherwise
    */
    bool _is_any_inverter_faulted();

    /**
     * @brief Method will check the drive enable flag per motor/inverter
     * @return True if all inverters are in the drive_enabled state, false otherwise
    */
    bool _is_drive_enabled();

    /**
     * @brief Method will check the current Drivetrain mode is the same as the current mode reported by the inverters
     * @return True if they match, false otherwise
    */
    bool DrivetrainSystem::is_control_mode_mismatched(unsigned long current_millis) const;

    /**
     * @brief Method will set the drive enable flag per motor/inverter.
     *        As redundancy, if the drive enable flag is going to be set low, we will also set the motor idle.
     *        This should happen automatically, but since it is untested, we will just add this redundancy feature now.
     * @param state is the state the flag will be set to
    */
    void _set_drive_enable_(bool state);

    /**
     * @brief Applies this tick's torque command to each corner by calling set_motors_torque for every inverter
    */
    void _set_drivetrain_torque(DrivetrainCommand_s command, unsigned long current_millis);

    /**
     * @brief Applies this tick's speed command to each corner by calling set_motors_speed for every inverter
    */
    void _set_drivetrain_speed(DrivetrainCommand_s command, unsigned long current_millis);

};

using DrivetrainInstance = etl::singleton<DrivetrainSystem>;

#endif /* DRIVETRAINSYSTEM */