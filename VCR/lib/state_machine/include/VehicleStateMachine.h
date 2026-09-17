#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "Logger.h"


class VehicleStateMachine
{
public:

    VehicleStateMachine(
        etl::delegate<void()> setMotorsIdle,
        etl::delegate<bool()> isVehicleLatched,
        etl::delegate<bool()> isRTDPressed,
        etl::delegate<void()> startBuzzer,
        etl::delegate<bool()> isBrakePressed,
        etl::delegate<bool()> isPedalsTimedOut,
        etl::delegate<void()> isSteeringTimedOut,
        etl::delegate<void()> resetPedalsHeartbeat,
        etl::delegate<void()> resetSteeringHeartbeat,
        etl::delegate<void()> sendRecalibratePedalsMessage,
        etl::delegate<void()> sendRecalibrateSteeringMessage,
        etl::delegate<bool()> isPedalsRecalibratePressed,
        etl::delegate<bool()> isSteeringRecalibratePressed,
        etl::delegate<bool()> isDrivetrainFaulted,
        etl::delegate<bool()> isDrivetrainNotConnected
    ) :
        _setMotorsIdle(setMotorsIdle),
        _isVehicleLatched(isVehicleLatched),
        _isRTDPressed(isRTDPressed),
        _startBuzzer(startBuzzer),
        _isBrakePressed(isBrakePressed),
        _isPedalsTimedOut(isPedalsTimedOut),
        _isSteeringTimedOut(isSteeringTimedOut),
        _resetPedalsHeartbeat(resetPedalsHeartbeat),
        _resetSteeringHeartbeat(resetSteeringHeartbeat),
        _isPedalsRecalibratePressed(isPedalsRecalibratePressed),
        _isSteeringRecalibratePressed(isSteeringRecalibratePressed),
        _sendRecalibratePedalsMessage(sendRecalibratePedalsMessage),
        _sendRecalibrateSteeringMessage(sendRecalibrateSteeringMessage),
        _isDrivetrainFaulted(isDrivetrainFaulted),
        _isDrivetrainNotConnected(isDrivetrainNotConnected)
    {
        _current_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    }

    VehicleState_e tickStateMachine(unsigned long curr_time_millis);

    VehicleState_e get_state() const { return _current_state; }

private:

    VehicleState_e _current_state;

    /**
     * Timestamp when entering WANTING_RECALIBRATE_PEDALS to ensure we stay there
     * for 1000ms before actually sending the recalibration command.
     */
    uint32_t _last_entered_pedals_waiting_state_ms = 0;
    /**
     * Timestamp when entering WANTING_RECALIBRATE_STEERING to ensure 3000ms pass before calibrating.
     */
    uint32_t _last_entered_steering_waiting_state_ms = 0;

    void _setState(VehicleState_e new_state, unsigned long current_time_millis);

    void _handleEntryLogic(VehicleState_e prev_state, unsigned long current_time_millis);

    void _handleExitLogic(VehicleState_e new_state, unsigned long current_time_millis);

    /**
     * Lambdas necessary for state machine to work.
     */
    etl::delegate<void()> _setMotorsIdle;
    etl::delegate<bool()> _isVehicleLatched;
    etl::delegate<bool()> _isRTDPressed;
    etl::delegate<void()> _startBuzzer;
    etl::delegate<bool()> _isBrakePressed;
    etl::delegate<bool()> _isPedalsTimedOut;
    etl::delegate<bool()> _isSteeringTimedOut;
    etl::delegate<void()> _resetPedalsHeartbeat;
    etl::delegate<void()> _resetSteeringHeartbeat;
    etl::delegate<bool()> _isPedalsRecalibratePressed;
    etl::delegate<bool()> _isSteeringRecalibratePressed;
    etl::delegate<void()> _sendRecalibratePedalsMessage;
    etl::delegate<void()> _sendRecalibrateSteeringMessage;
    etl::delegate<bool()> _isDrivetrainFaulted;
    etl::delegate<bool()> _isDrivetrainNotConnected;

};

using VehicleStateMachineInstance = etl::singleton<VehicleStateMachine>;

#endif // VEHICLE_STATE_MACHINE_H
