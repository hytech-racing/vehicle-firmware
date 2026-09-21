#ifndef DRIVEBRAINCONTROLLER_H
#define DRIVEBRAINCONTROLLER_H

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <cstdint>
#include <cmath>

/* Local System Includes */
#include "DrivetrainSystem.h"

/* Local Controller Includes */
#include "controllers/SimpleController.h"

#define WORST_LATENCY_PERIOD_MS 1000


struct MessageLatencyInfo_s
{
    bool has_timing_failure;
    unsigned long worst_period_millis;
};

struct DrivebrainControllerParams_s
{
    unsigned long allowed_latency;
    ControllerMode_e assigned_controller_mode;
};

class DrivebrainController
{
public:

    /**
     * @brief Constructor for the drivebrain controller class
     * @param allowed_latency_ms the allowed latency in milliseconds. If the most recent packet has
     *                           a timestamp older than this measure of time, then we fail safe
     * @param assigned_controller_mode the controller mode that the drivebrain controller is assigned to
     *                                 Is required for evaluating whether or not we are active or not
    */
    explicit DrivebrainController(unsigned long allowed_latency_ms,
                                ControllerMode_e assigned_controller_mode = ControllerMode_e::MODE_4
    ) : _emergency_control()
    {
        _params = {
            allowed_latency_ms,
            assigned_controller_mode
        };
    }

    /// @brief evaluate function for running the business logic
    /// @param state the current state of the car
    /// @param curr_millis
    /// @return torque controller output that gets passed through the TC MUX
    DrivetrainCommand_s evaluate(const VCRData_s &curr_state, unsigned long curr_millis);

    MessageLatencyInfo_s getRAUXLatencyInfo() { return _raux_latency_info; }

    MessageLatencyInfo_s getTELEMLatencyInfo() { return _telem_latency_info; }

    /// @brief getter for the current status of whether or not the controller has had a timing failure during operation
    /// @return bool of status
    bool hasTimingFailure() const { return !_should_run_controller; }

private:

    DrivebrainControllerParams_s _params;
    bool _should_run_controller = true;
    bool _last_reset_worse_latency_clock = 0;
    MessageLatencyInfo_s _raux_latency_info = { false, 0 };
    MessageLatencyInfo_s _telem_latency_info = { false, 0 };
    SimpleTorqueController _emergency_control {{1.0f, 1.0f, 20000.0f, 10.0f, -15.0f}}; // NOLINT



    /**
     * @brief Method check if there is a "timing failure", which means message are too latent or have not been received
     * @note There are 3 cases, which are explained in the method definition
    */
    void _checkDrivebrainCommandTimingFailure(StampedDrivetrainCommand_s command,
                                            unsigned long curr_millis,
                                            MessageLatencyInfo_s& latency_info
    );

};

#endif // DRIVEBRAINCONTROLLER_H