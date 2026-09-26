#include "VehicleStateMachine.hpp"


VehicleState_e VehicleStateMachine::tickStateMachine(unsigned long current_millis)
{
    switch (_current_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            /**
             * @brief Serves as the startup/default state: LV on and/or TSMS on
             * @note TRACTIVE_SYSTEM_NOT_ACTIVE just means that we have not LATCHED
             *
             * ERROR MODES :
             *  - If DTSM in NOT_CONNECTED -> VSM goes to ERROR since we have lost communication with inverters
             *  - If DTSM in in FAULTED -> VSM will stay in current state since FAULTED is not a fatal issue
            */

            // Error mode(s) checking
            if (_isDrivetrainNotConnected())
            {
                _setState(VehicleState_e::ERROR, current_millis);
                break;
            }

            if (_isDrivetrainFaulted())
            {
                // If we are faulted, then we will not allow any recalibration
                break;
            }

            // Check for latch state change
            if (_isVehicleLatched())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            // Check for recalibration state changes
            if (_isPedalsRecalibratePressed())
            {
                _setState(VehicleState_e::WANTING_RECALIBRATE_PEDALS, current_millis);
                break;
            }

            if (_isSteeringRecalibratePressed())
            {
                _setState(VehicleState_e::WANTING_RECALIBRATE_STEERING, current_millis);
                break;
            }

            // Set drivetrain idle for safety
            _setMotorsIdle();
            _handleDrivetrainCommand(false, current_millis); // what does this do?

            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        {
            /**
             * @brief State when vehicle is LATCHED
             * @note TRACTIVE_SYSTEM_NOT_ACTIVE just means that we have not LATCHED
             * @note We will not allow steering or pedals recalibration in this state since they should happen rarely, and
             *       only when car is not moving/we are settin up the car pre-race
             *
             * ERROR MODES :
             *  - If DTSM in NOT_CONNECTED -> VSM goes to ERROR since we have lost communication with inverters
             *  - If DTSM in in FAULTED -> VSM will stay in current state since FAULTED is not a fatal issue
             *  - If vehicle becomes unlatched, return to TRACTIVE_SYSTEM_NOT_ACTIVE
            */

            // Error mode(s) checking
            if (_isDrivetrainNotConnected())
            {
                _setState(VehicleState_e::ERROR, current_millis);
                break;
            }

            if (_isDrivetrainFaulted())
            {
                // If we are faulted, then we will not allow any recalibration
                break;
            }

            if (!_isVehicleLatched())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            // Check for RTD state change
            if (_isRTDPressed() && _isBrakePressed() && _isDrivetrainFaulted())
            {
                _setState(VehicleState_e::READY_TO_DRIVE, current_millis);
                break;
            }

            break;
        }
        case VehicleState_e::READY_TO_DRIVE:
        {
            /**
             * @brief Vehicle can move at this state (LATCHED and RTD pressed)
             * @note We will not allow steering or pedals recalibration in this state since they should happen rarely, and
             *       only when car is not moving/we are settin up the car pre-race
             *
             * ERROR MODES :
             *  - If DTSM in NOT_CONNECTED -> VSM goes to ERROR since we have lost communication with inverters
             *  - If DTSM in in FAULTED -> Only lose RTD, stay latched (not fatal issue)
             *  - If vehicle becomes unlatched, return to TRACTIVE_SYSTEM_NOT_ACTIVE
             *  - If pedals heartbeat timesout/misses -> Only lose RTD, stay latched (not HV issue)
             *  - If steering heartbeat timesout/misses -> Only lose RTD, stay latched (not HV issue)
            */

            _handleDrivetrainCommand(true, current_millis);

            // Error mode(s) checking
            if (_isDrivetrainNotConnected())
            {
                _setState(VehicleState_e::ERROR, current_millis);
                break;
            }

            if (_isDrivetrainFaulted())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if (!_isVehicleLatched())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_isPedalsTimedOut())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if (_isSteeringTimedOut())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
            }

            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            /**
             * @brief State is used to ensure that pedals recalibration is wanted, not just accidental
             * @note This serves as software debounce protection
             * @note We will only allow recalibration during TRACTIVE_SYSTEM_NOT_ACTIVE, thus we can just return there
             *
             * ERROR MODES :
             *  - If DTSM in NOT_CONNECTED -> VSM goes to ERROR since we have lost communication with inverters
            */

            // Error mode(s) checking
            if (_isDrivetrainNotConnected())
            {
                _setState(VehicleState_e::ERROR, current_millis);
                break;
            }

            // Accidental press
            if (!_isPedalsRecalibratePressed())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            // Actual press + debounce
            if (_isPedalsRecalibratePressed() && (current_millis - _last_entered_pedals_waiting_state_ms > 3000))
            {
                _setState(VehicleState_e::RECALIBRATING_PEDALS, current_millis);
            }

            _handleDrivetrainCommand(false, current_millis);
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            /**
             *
            */

            _handleDrivetrainCommand(false, current_millis);

            if (!_isSteeringRecalibratePressed())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_isSteeringRecalibratePressed() && (current_millis - _last_entered_steering_waiting_state_ms > 3000))
            {
                _setState(VehicleState_e::RECALIBRATING_STEERING, current_millis);
            }

            break;
        }
        case VehicleState_e::RECALIBRATING_PEDALS:
        {
            _handleDrivetrainCommand(false, current_millis);

            if (!_isPedalsRecalibratePressed())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_isPedalsRecalibratePressed())
            {
                _sendRecalibratePedalsMessage();
            }

            break;
        }
                case VehicleState_e::RECALIBRATING_STEERING:
         {
            /**
             * @note Set motors idle for safety
             * @note Only leave the
            */

            _handleDrivetrainCommand(false, current_millis);

            if (!_isSteeringRecalibratePressed())
            {
                _setState(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_isSteeringRecalibratePressed())
            {
                _sendRecalibrateSteeringMessage();
            }

            break;
        }
        default:
        {
            break;
        }
    }
    return _current_state;
}

void VehicleStateMachine::_setState(VehicleState_e new_state, unsigned long curr_millis)
{
    _handleExitLogic(_current_state, curr_millis);
    _current_state = new_state;
    _handleEntryLogic(_current_state, curr_millis);
}

void VehicleStateMachine::_handleExitLogic(VehicleState_e prev_state, unsigned long curr_millis)
{
    switch (prev_state)
    {
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::RECALIBRATING_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            _last_entered_steering_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::RECALIBRATING_STEERING:
        {
            _last_entered_steering_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        case VehicleState_e::READY_TO_DRIVE:
        default:
            break;
    }
}

void VehicleStateMachine::_handleEntryLogic(VehicleState_e new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
        case VehicleState_e::READY_TO_DRIVE:
        {
            _startBuzzer();
            _resetPedalsHeartbeat();
            _resetSteeringHeartbeat();
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = curr_millis;
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            _last_entered_steering_waiting_state_ms = curr_millis;
            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        case VehicleState_e::RECALIBRATING_STEERING:
        case VehicleState_e::RECALIBRATING_PEDALS:
        default:
            break;
    }
}

