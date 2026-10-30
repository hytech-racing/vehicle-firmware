#ifndef CONTROLS_IMPL
#define CONTROLS_IMPL

#include "VCR_Globals.hpp"

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "VCRCANInterfaceImpl.hpp"

/* Local System Includes */
#include "DrivetrainSystem.hpp"
#include "TorqueControllerMux.hpp"
#include "controllers/DrivebrainController.hpp"
#include "controllers/LoadCellVectoringTorqueController.hpp"
#include "controllers/SimpleController.hpp"
#include "controllers/SimpleLaunchController.hpp"


// Regen has to be fully cut before the bus reaches the voltage the inverters treat as the top of the HV range
static_assert(
    tc_mux_default_params::REGEN_VOLTAGE_CUTOFF_V <= dti_default_params::MAXIMUM_HV_VOLTAGE,
    "REGEN_VOLTAGE_CUTOFF_V must not exceed dti_default_params::MAXIMUM_HV_VOLTAGE");
// The inverter-level cutoff is the hard guarantee for EV.3.3.3, so it has to sit at or above the rules speed on its own
static_assert(
    dti_default_params::BRAKE_CURRENT_CUTOFF_KPH >= tc_mux_default_params::RULES_MIN_REGEN_SPEED_KPH,
    "EV.3.3.3: inverter brake current cutoff must be at or above 5 km/h");
static_assert(
    dti_default_params::BRAKE_CURRENT_REENABLE_KPH > dti_default_params::BRAKE_CURRENT_CUTOFF_KPH,
    "brake current re-enable speed must be above the cutoff for hysteresis");

class VCRControls
{
  public:
    VCRControls() = delete;

    /**
     * @brief Explicit constructor that passes in a pointer to an already-instantiated DrivetrainSystem
     * @param max_allowed_db_latency_ms The maximum allowed latency between commands from drivebrain before considering the connection invalid
     */
    explicit VCRControls(DrivetrainSystem* dt_system, uint32_t max_allowed_db_latency_ms)
        : _mode4(max_allowed_db_latency_ms),
          _tc_mux(
              {[this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
               [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode1.evaluate(state, curr_millis); },
               [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
               [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode3.evaluate(state, curr_millis); },
               [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s
               {
                   return _mode4.evaluate(state, curr_millis);
               }},
              {false, false, false, false, true}),
          _dt_system(dt_system) {};

    /**
     * @brief Primary function in VCRControls, allowing us to command the drivetrain based on vehicle state
     * @note After the drivetrain state machine determines that the drivetrain must be commanded, it invokes
     *       this function, which will find the function from the tc_mux and invoke the correct one on the drivetrain system
     * @param is_drive_enable_requested true only while the VSM is in READY_TO_DRIVE
     */
    void handleDrivetrainCommand(bool is_drive_enable_requested, unsigned long curr_millis);

    /**
     * @note Drivebrain is considered in control if there is no latency/timing failure and if we are running mode 4
     * @return True if in control, false otherwise
     */
    bool isDrivebrainInControll() const;

    void enqueueLatencyCANData() const;

    /**
     * @brief This is a wrapper for TCMux's method getTCMuxStatus()
     * @note We wrap TorqueControllerMux's getter because it is not a singleton. _tc_mux only exists as a private member
     *       owned by VCRControls, so there is no global instance to call getTCMuxStatus() on directly from outside this class
     */
    TorqueControllerMuxStatus_s getTCMuxStatus() const { return _tc_mux.getTCMuxStatus(); }

    /**
     * @note Controllers (_mode0, _mode1, _mode3, _mode4) are plain private members of
     *       VCRControls, not singletons. To limit access, each passthrough below exposes only
     *       specific information that outside code would need.
     */
    LaunchStates_e getLaunchState() const { return _mode3.get_launch_state(); };
    bool getHasTimingFailure() const { return _mode4.hasTimingFailure(); };
    MessageLatencyInfo_s getRAUXLatencyInfo() const { return _mode4.getRAUXLatencyInfo(); };
    MessageLatencyInfo_s getTELEMLatencyInfo() const { return _mode4.getTELEMLatencyInfo(); };

  private:
    SimpleTorqueController _mode0; // this needs to be first for tc_mux to have a valid capture
    LoadCellVectoringTorqueController _mode1;
    SimpleLaunchController _mode3;
    DrivebrainController _mode4;
    TCMuxType _tc_mux;
    DrivetrainSystem* _dt_system = nullptr;
    DrivetrainCommand_s _debug_dt_command = {};
};

using VCRControlsInstance = etl::singleton<VCRControls>;

#endif