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

    /**
     * Explicit constructor that passes in a pointer to an already-instantiated DrivetrainSystem.
     * @param max_allowed_db_latency_ms The maximum allowed latency between commands from the
     *                                  DriveBrain before considering the connection invalid.
     */
    explicit VCRControls(DrivetrainSystem *dt_system, uint32_t max_allowed_db_latency_ms);

    DrivetrainCommand_s _debug_dt_command = {};

    /**
     * Primary function in VCRControls. After the drivetrain state machine determines that
     * the drivetrain must be commanded, it invokes this function, which will find the
     * function from the tc_mux and invoke the correct one on the drivetrain system.
     */
    void handle_drivetrain_command(bool wanting_ready_to_drive, bool ready_to_drive);

    bool drivebrain_is_in_control() const;

    bool drivebrain_timing_failure() const;

    void send_controls_can_messages();

    /**
     * Function to cycle to the next torque limit (low, mid, max). The button input must
     * be handled elsewhere.
     */
    void cycle_torque_limit()
    {
        size_t torque_limit_int = static_cast<size_t>(_torque_limit);
        size_t new_torque_limit = (torque_limit_int + 1) % (static_cast<size_t>(TorqueLimit_e::TCMUX_NUM_TORQUE_LIMITS));
        _torque_limit = static_cast<TorqueLimit_e>(new_torque_limit);
    }

    TorqueLimit_e get_current_torque_limit() { return _torque_limit; }

    SimpleLaunchController& get_launch_controller() { return _mode3; }

    TorqueControllerMuxStatus_s get_tc_mux_status() const { return _tc_mux.get_tc_mux_status(); }

private:

    TorqueControllerSimple _mode0; // this needs to be first for tc_mux to have a valid capture
    LoadCellVectoringTorqueController _mode1;
    SimpleLaunchController _mode3;
    DrivebrainController _mode4;
    TCMuxType _tc_mux;
    TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_FULL_TORQUE;
    DrivetrainSystem *_dt_system = nullptr;

};

using VCRControlsInstance = etl::singleton<VCRControls>;

#endif