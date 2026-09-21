#ifndef CONTROLS_IMPL
#define CONTROLS_IMPL

#include "VCR_Globals.h"

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "VCRCANInterfaceImpl.h"

/* Local System Includes */
#include "DrivetrainSystem.h"
#include "TorqueControllerMux.hpp"
#include "controllers/SimpleController.h"
#include "controllers/LoadCellVectoringTorqueController.h"
#include "controllers/SimpleLaunchController.h"
#include "controllers/DrivebrainController.h"


class VCRControls
{
public:

    VCRControls() = delete;

    /**
     * @brief Explicit constructor that passes in a pointer to an already-instantiated DrivetrainSystem
     * @param max_allowed_db_latency_ms The maximum allowed latency between commands from drivebrain before considering the connection invalid
    */
    explicit VCRControls(DrivetrainSystem *dt_system,
                        uint32_t max_allowed_db_latency_ms
    ) :
        _mode4(max_allowed_db_latency_ms),
        _tc_mux({
            [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
            [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode1.evaluate(state, curr_millis); },
            [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
            [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode3.evaluate(state, curr_millis); },
            [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode4.evaluate(state, curr_millis); }
        },
        {false, false, false, false, true}),
        _dt_system(dt_system)
    {};

    DrivetrainCommand_s _debug_dt_command = {};

    /**
     * @brief Primary function in VCRControls, allowing us to command the drivetrain based on vehicle state
     * @note After the drivetrain state machine determines that the drivetrain must be commanded, it invokes
     *       this function, which will find the function from the tc_mux and invoke the correct one on the drivetrain system.
    */
    void handleDrivetrainCommand(bool ready_to_drive, unsigned long curr_millis);

    /**
     * @note Drivebrain is considered in control if there is no latency/timing failure and if we are running mode 4
     * @return True if in control, false otherwise
    */
    bool isDrivebrainInControll() const;

    void enqueueLatencyCANData();

    TorqueLimit_e getCurrentTorqueLimit() { return _torque_limit; }


    /**
     * @brief This is a wrapper for TC Mux's method getTCMuxStatus()
     * @note We wrap TorqueControllerMux's getter because it is not a singleton. _tc_mux only exists as a private member
     *       owned by VCRControls, so there is no global instance to call getTCMuxStatus() on directly from outside this class
    */
    TorqueControllerMuxStatus_s getTCMuxStatus() const { return _tc_mux.getTCMuxStatus(); }

    /* ---------- Wrapper Methods For Controllers ---------- */
    /**
     * @note Controllers (_mode0, _mode1, _mode3, _mode4) are plain private members of
     *       VCRControls, not singletons. To limit access, each passthrough below exposes exactly
     *       one specific, read-only piece of information that outside code would need. Prohibit
     *       access to calling "evaluate" for any controller
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
    TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_FULL_TORQUE;
    DrivetrainSystem *_dt_system = nullptr;

};

using VCRControlsInstance = etl::singleton<VCRControls>;

#endif