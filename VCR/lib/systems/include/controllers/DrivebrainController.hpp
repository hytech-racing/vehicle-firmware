#ifndef DRIVEBRAINCONTROLLER_H
#define DRIVEBRAINCONTROLLER_H

/* External Includes */
#include "SharedFirmwareTypes.h"

#include <cmath>
#include <cstdint>

/* Local System Includes */
#include "DrivetrainSystem.hpp"

/* Local Controller Includes */
#include "controllers/SimpleController.hpp"

#define WORST_LATENCY_PERIOD_MS 1000


struct MessageLatencyInfo_s
{
    bool timing_failure;
    unsigned long worst_period_millis;
};

class DrivebrainController
{
  public:
    /// @brief constructor for the drivebrain controller class
    /// @param allowed_latency the allowed latency in milliseconds for which if the most recent packet has a timestamp older than this measure of time we fail
    /// safe
    /// @param assigned_controller_mode the controller mode that the drivebrain controller is assigned to. is required for evaluating whether or not we are
    /// active or not
    explicit DrivebrainController(unsigned long allowed_latency, ControllerMode_e assigned_controller_mode = ControllerMode_e::MODE_4)
        : _emergency_control()
    {
        _params = {allowed_latency, assigned_controller_mode};
    }

    /// @brief evaluate function for running the business logic
    /// @param state the current state of the car
    /// @param curr_millis
    /// @return torque controller output that gets passed through the TC MUX
    DrivetrainCommand_s evaluate(const VCRData_s& state, unsigned long curr_millis);

    MessageLatencyInfo_s getAuxLatencyData() { return _aux_latency_info; }

    MessageLatencyInfo_s getTelemLatencyData() { return _telem_latency_info; }

    /// @brief getter for the current status of whether or not the controller has had a timing failure during operation
    /// @return bool of status
    bool getTimingFailureStatus() const { return !_should_run_controller; }

  private:
    struct
    {
        unsigned long allowed_latency = {};
        ControllerMode_e assigned_controller_mode = {};
    } _params;

    bool _should_run_controller = true;
    bool _last_reset_worse_latency_clock = 0;
    MessageLatencyInfo_s _aux_latency_info = {false, 0};
    MessageLatencyInfo_s _telem_latency_info = {false, 0};

    TorqueControllerSimple _emergency_control = {{1.0f, 1.0f, 20000.0f, 10.0f, -15.0f}}; // NOLINT

    void _checkDrivebrainCommandTimingFailure(StampedDrivetrainCommand_s command, unsigned long curr_millis, MessageLatencyInfo_s& latency_info);
};

#endif // DRIVEBRAINCONTROLLER_H
