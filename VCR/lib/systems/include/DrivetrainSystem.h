#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

/* Standard Library */
#include <stdint.h>

/* ETL Library */
#include <etl/variant.h>
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include <array>
#include <functional>
#include <SysClock.h>
#include "shared_types.h"
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "SystemTimeInterface.h"


/**
 * When user calls evaluate_drivetrain(), this is part of the returned status to
 * indicate if the command was successful or invalid
 */
enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,
    CANNOT_INIT_NOT_CONNECTED = 1, // When requesting init but inverters are not yet requested
    COMMAND_INVALID = 2
};

/**
 * Actual struct that gets returned on drivetrain evaluation. Contains the current DSM state,
 * the command response (OK, INVALID, etc), and each corner's inverter status.
 */
struct DrivetrainStatus_s
{
    bool all_inverters_connected;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_resp;
    DrivetrainState_e state;
};

/**
 * There are three types of commands going into the DrivetrainSystem. There is the
 * normal DrivetrainCommand (see SharedFirmwareTypes.h), a "reset error" command,
 * and a "init" command.
 */
struct DrivetrainResetError_s
{
    bool reset_errors; // true: reset the errors present on inverters, false: dont
};

enum DrivetrainModeRequest_e
{
    UNINITIALIZED = 0, // If sending a DrivetrainInit command with UNIITIALIZED, it will not initialize
    INIT_DRIVE_MODE = 1
};

struct DrivetrainInit_s
{
    DrivetrainModeRequest_e init_drivetrain;
};

/**
 * The DrivetrainSystem is primarily responsible for two things:
 * 1) Updating its internal state machine
 * 2) Determining what commands to give each InverterInterface
 */

class DrivetrainSystem
{
public:

    /**
     * etl::variants allow multiple types to be treated as a single type-- almost like an enum of types.
     * Here, we're just saying that when we refer to CmdVariant, the parameter can be any one of these
     * three options.
     */
    using CmdVariant = etl::variant<DrivetrainCommand_s, DrivetrainInit_s, DrivetrainResetError_s>;
    DrivetrainSystem() = delete;

    /**
     * Functions for VSM state transitions (VSM needs to know drivetrain's status to trigger its
     * state transitions).
     */
    bool hv_over_threshold();
    bool drivetrain_error_present();
    bool drivetrain_ready();
    void reset_dt_error();

    /**
     * Drivetrain state machine (DSM) functions
     */
    DrivetrainStatus_s evaluate_drivetrain(CmdVariant cmd);
    DrivetrainState_e get_state() const;
    DrivetrainStatus_s get_status() const;

    struct InverterFuncts_s
    {
        std::function<void(float desired_rpm, float torque_limit_nm)> set_speed;
        std::function<void()> set_idle;
        std::function<void(InverterControlWord_s control_word)> set_inverter_control_word;
        std::function<InverterStatus_s()> get_status;
        std::function<MotorMechanics_s()> get_motor_mechanics;
    };

    DrivetrainSystem(veh_vec<DrivetrainSystem::InverterFuncts_s> inverter_interfaces, etl::delegate<void(bool)> set_ef_active_pin, unsigned long ef_pin_enable_delay_ms = 50); //why not make delay a constant that can easily be changed elsewhere where other constants are changed

private:

    /**
     * Internal functions for handling DSM state transitions.
     */
    bool _check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func);
    bool _drivetrain_active(float min_active_rpm);
    void _set_state(DrivetrainState_e state);
    void _set_drivetrain_disabled();
    void _set_drivetrain_keepalive_idle();
    void _set_enable_drivetrain_hv();
    void _set_enable_drivetrain();
    void _set_drivetrain_error_reset();
    void _set_drivetrain_command(DrivetrainCommand_s cmd);

    DrivetrainState_e _evaluate_state_machine(CmdVariant cmd);
    DrivetrainState_e _state;
    DrivetrainStatus_s _status;

    const float _active_rpm_level = 100;
    veh_vec<InverterFuncts_s> _inverter_interfaces;

    /**
     * Lambda functions defined on construction for the DSM state transitions.
     */
    std::function<bool(const InverterStatus_s &)> _check_inverter_ready_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_connected_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_quit_dc_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_no_errors_present;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_not_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_enabled;

    /**
     * Delegate function for setting ef active
     */
    etl::delegate<void(bool)> _set_ef_active_pin;
    unsigned long _last_toggled_ef_active = 0;
    unsigned long _ef_pin_enable_delay_ms;
    unsigned long _precharge_wait_start = 0;
    
};

using DrivetrainInstance = etl::singleton<DrivetrainSystem>;

#endif /* DRIVETRAINSYSTEM */