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
 *        It indicates whether the most recent command was actually successful or invalid given
 *        the drivetrain's current state.
*/
enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,                // Command was valid and has been applied.
    CANNOT_SEND_NOT_CONNECTED = 1, // Command was rejected because inverters are not yet connected.
    COMMAND_INVALID = 2            // Reserved for other invalid-command cases (currently unused — see note below).
};

/**
 * @brief Struct that gets returned on drivetrain evaluations. This is how users can determine
 *        what the drivetrain system is doing and whether the last command was accepted/successful.
 *
 * @param are_all_inverters_connected true only if all four inverters currently report `connected`.
 * @param are_all_inverters_enabled true only if all four inverters have confirmed drive-enable (i.e. echoed is_drive_enabled = true)
 * @param inverter_statuses per-corner status detail (FL/FR/RL/RR), so callers can diagnose which corner is disconnected/faulted/etc
 * @param cmd_response Whether the command just passed into evaluate_drivetrain() was successful or invalid
 * @param current_dsm_state
*/
struct DrivetrainStatus_s
{
    bool are_all_inverters_connected;
    bool are_all_inverters_enabled;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_response;
    DrivetrainState_e current_dsm_state;
};

/**
 * For now, I will not implement a way to reset the fault codes.
 * I think the most bare bones way is to set the fault code time to essentially zero?
 * However, I don't think that will necessarily work and I don't know if its even necessary.
 * We can see how often we get errors
*/

/**
 * @brief This struct bundles various hardware-agnostic callbacks DrivetrainSystem uses to talk
 *        to the inverters. NOTE: The system should never "see" the interface.
 *
 */
struct InverterInterfaceFuncts_s
{
    etl::delegate<void(float torque_nm)> set_torque;
    etl::delegate<void()> set_idle;
    etl::delegate<void(bool)> request_enable;
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

    DrivetrainSystem(veh_vec<InverterInterfaceFuncts_s> inverter_interfaces,
                    etl::delegate<bool()> is_hv_status_ok
    ) :
        _current_state(DrivetrainState_e::NOT_CONNECTED),
        _inverter_interfaces(inverter_interfaces),
        _is_hv_status_ok(is_hv_status_ok)
    {}



    // ----------  DRIVETRAIN INTERNAL STATE MACHINE FUNCTIONS ---------- //
    /**
     * @brief This method fills out the DrivetrainStatus_s struct (defined above).
    */
    DrivetrainStatus_s evaluate_drivetrain(DrivetrainCommand_s cmd, unsigned long current_millis);

    // Getters
    DrivetrainState_e get_current_state() const;
    DrivetrainStatus_s get_status() const;
    const char* get_state_name() const;


    // ----------  VEHICLE STATE MACHINE FUNCTIONS ---------- //
    // Functions for VSM state transitions (VSM needs to know drivetrain's status to trigger its state transitions.

    // See what the vehicle state machine actually needs then we can build this out.

private:

    /**
     * @brief
    */
    DrivetrainState_e _evaluate_state_machine(DrivetrainCommand_s cmd, unsigned long current_millis);

    //  Internal functions for handling DSM state transitions.
    void _set_state(DrivetrainState_e new_state, unsigned long current_millis);
    void _handle_exit_logic(DrivetrainState_e prev_state, unsigned long current_millis);
    void _handle_entry_logic(DrivetrainState_e new_state, unsigned long current_millis);

    /**
     * @brief This method checks if we are connected with each of the inverters.
     *        Connection in this case is defined as we are continously receiving CAN messages.
     * @return true if all inverters are connected, else false
    */
    bool _are_all_inverters_connected();

    /**
     * @brief This method checks if any of the 4 inverters (FL, FR, RL, RR) have a fault code
     * @return true if any inverter has a fault code, else false
     */
    bool _is_any_inverter_faulted();

    /**
     * @brief This method will check the drive enable flag per motor/inverter.
    */
    bool _is_drive_enabled();

    /**
     * @brief This method will set the drive enable flag per motor/inverter.
     *        As redundancy, if the drive enable flag is going to be set low, we will also set the motor idle.
     *        This should happen automatically, but since it is untested, we will just add this redundancy feature now.
     * @param state is the state the flag will be set to
    */
    void _set_drive_enable_(bool state);

    /**
     * @brief TODO FILL OUT
    */
    void _set_drivetrain_command(DrivetrainCommand_s cmd);


    DrivetrainState_e _current_state;
    DrivetrainStatus_s _status;
    unsigned long _last_state_changed_time = 0;
    veh_vec<InverterInterfaceFuncts_s> _inverter_interfaces;

    /**
     * @brief Checks DC bus voltage (and possibly duty cycle) reported by the
     *        inverters is within expected thresholds. The inverters do their own
     *        checking internally; this is our own software redundancy on top.
     *        Defined/bound in VCR_SystemTasks, not inside DrivetrainSystem.
     */
    etl::delegate<bool()> _is_hv_status_ok;

};

using DrivetrainInstance = etl::singleton<DrivetrainSystem>;

#endif /* DRIVETRAINSYSTEM */