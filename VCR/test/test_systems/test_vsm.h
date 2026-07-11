#include "VehicleStateMachine.h"
#include <gtest/gtest.h>

bool hv_over_threshold = false;
bool start_btn = false;
bool brake_pressed = false;
bool drivetrain_error = false;
bool drivetrain_ready = false;
bool wanting_ready_to_drive = false;
bool ready_to_drive = false;
bool buzzer_active = false;
bool pedals_timeout = false;
bool called_recalibrate_pedals = false;
bool inverter_button_pressed = false;
bool calibrate_pedals_pressed = false;
bool called_reset_inverter_error = false;
bool called_recalibrate_steering = false;
bool calibrate_steering_pressed = false;
bool steering_timeout = false;

void reset_all_booleans()
{
    hv_over_threshold = false;
    start_btn = false;
    brake_pressed = false;
    drivetrain_error = false;
    drivetrain_ready = false;
    wanting_ready_to_drive = false;
    ready_to_drive = false;
    buzzer_active = false;
    pedals_timeout = false;
    called_recalibrate_pedals = false;
    inverter_button_pressed = false;
    calibrate_pedals_pressed = false;
    called_reset_inverter_error = false;
    called_recalibrate_steering = false;
    calibrate_steering_pressed = false;
    steering_timeout = false;
}

etl::delegate<bool()> mock_hv_over_threshold = etl::delegate<bool()>::create([]() -> bool {
    return hv_over_threshold;
});

etl::delegate<bool()> mock_start_btn = etl::delegate<bool()>::create([]() -> bool {
    return start_btn;
});

etl::delegate<bool()> mock_brake_pressed = etl::delegate<bool()>::create([]() -> bool {
    return brake_pressed;
});

etl::delegate<bool()> mock_drivetrain_error = etl::delegate<bool()>::create([]() -> bool {
    return drivetrain_error;
});

etl::delegate<bool()> mock_drivetrain_ready = etl::delegate<bool()>::create([]() -> bool {
    return drivetrain_ready;
});

etl::delegate<void()> mock_start_buzzer = etl::delegate<void()>::create([]() -> void {
    buzzer_active = true;
});

etl::delegate<bool()> mock_buzzer_done = etl::delegate<bool()>::create([]() -> bool {
    return !buzzer_active;
});

etl::delegate<void()> mock_end_buzzer = etl::delegate<void()>::create([]() -> void {
    buzzer_active = false;
});

etl::delegate<void()> mock_handle_drivetrain_init = etl::delegate<void()>::create([]() -> void {
    return;
});

void mock_command_drivetrain_function(bool wrtd, bool rtd)
{
    wanting_ready_to_drive = wrtd;
    ready_to_drive = rtd;
    return;
}
etl::delegate<void(bool, bool)> mock_command_drivetrain = etl::delegate<void(bool, bool)>::create<mock_command_drivetrain_function>();

etl::delegate<bool()> mock_pedals_timeout = etl::delegate<bool()>::create([]() -> bool {
    return pedals_timeout;
});

etl::delegate<void()> mock_pedals_reset = etl::delegate<void()>::create([]() -> void {
    pedals_timeout = false;
    return;
});

etl::delegate<void()> mock_recalibrate_pedals = etl::delegate<void()>::create([]() -> void {
    called_recalibrate_pedals = true;
    return;
});

etl::delegate<bool()> mock_inverter_button_pressed = etl::delegate<bool()>::create([]() -> bool {
    return inverter_button_pressed;
});

etl::delegate<bool()> mock_calibrate_pedals_pressed = etl::delegate<bool()>::create([]() -> bool {
    return calibrate_pedals_pressed;
});

etl::delegate<void()> mock_reset_inverter_error = etl::delegate<void()>::create([]() -> void {
    called_reset_inverter_error = true;
    return;
});

etl::delegate<void()> mock_recalibrate_steering = etl::delegate<void()>::create([]() -> void {
    called_recalibrate_steering = true;
    return;
});

etl::delegate<bool()> mock_is_calibrate_steering_pressed = etl::delegate<bool()>::create([]() -> bool {
    return calibrate_steering_pressed;
});

etl::delegate<bool()> mock_steering_timeout = etl::delegate<bool()>::create([]() -> bool {
    return steering_timeout;
});

etl::delegate<void()> mock_steering_reset = etl::delegate<void()>::create([]() -> void {
    steering_timeout = false;
    return;
});

void ASSERT_DRIVETRAIN_COMMANDED(bool wrtd, bool rtd)
{
    ASSERT_EQ(wanting_ready_to_drive, wrtd);
    ASSERT_EQ(ready_to_drive, rtd);
}

TEST (VehicleStateMachine, TractiveSystemNotActive) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Ensure errors can be cleared
    inverter_button_pressed = true;
    drivetrain_error = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(called_reset_inverter_error, false);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    inverter_button_pressed = false;
    drivetrain_error = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(called_reset_inverter_error, false);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    inverter_button_pressed = true;
    drivetrain_error = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_EQ(called_reset_inverter_error, true);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);
    called_reset_inverter_error = false;
    
}

// TODO: Add CalibratingSteering test

TEST (VehicleStateMachine, CalibratingPedals) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Enter WANTING_RECALIBRATE_PEDALS
    calibrate_pedals_pressed = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Leave WANTING_RECALIBRATE_PEDALS because not enough time has passed
    calibrate_pedals_pressed = false;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Enter WANTING_RECALIBRATE_PEDALS
    calibrate_pedals_pressed = true;
    state_machine.tick_state_machine(2000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Not enough time has passed
    calibrate_pedals_pressed = true;
    state_machine.tick_state_machine(4999);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Entered RECALIBRATING_PEDALS
    calibrate_pedals_pressed = true;
    state_machine.tick_state_machine(5001);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::RECALIBRATING_PEDALS);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Called calibrate pedals function
    calibrate_pedals_pressed = true;
    state_machine.tick_state_machine(6000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::RECALIBRATING_PEDALS);
    ASSERT_EQ(called_recalibrate_pedals, true);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);
    called_recalibrate_pedals = false;
    state_machine.tick_state_machine(6700);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::RECALIBRATING_PEDALS);
    ASSERT_EQ(called_recalibrate_pedals, true);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);
    called_recalibrate_pedals = false;

    // Exit RECALIBRATING_PEDALS
    calibrate_pedals_pressed = false;
    state_machine.tick_state_machine(7000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);
    
}

TEST (VehicleStateMachine, TractiveSystemActive) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(115);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Ensure errors can be cleared
    inverter_button_pressed = true;
    drivetrain_error = false;
    state_machine.tick_state_machine(254);
    ASSERT_EQ(called_reset_inverter_error, false);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    inverter_button_pressed = false;
    drivetrain_error = true;
    state_machine.tick_state_machine(670);
    ASSERT_EQ(called_reset_inverter_error, false);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    inverter_button_pressed = true;
    drivetrain_error = true;
    state_machine.tick_state_machine(841);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_EQ(called_reset_inverter_error, true);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);
    called_reset_inverter_error = false;

    // Returns to TRACTIVE_SYSTEM_NOT_ACTIVE when hv dips below threshold
    hv_over_threshold = false;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    hv_over_threshold = true;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Only goes into WRTD when start button & brake pressed
    start_btn = true;
    brake_pressed = false;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    start_btn = false;
    brake_pressed = true;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

}

TEST (VehicleStateMachine, WantingReadyToDrive) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into WANTING_READY_TO_DRIVE
    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, false);

    // Returns to TRACTIVE_SYSTEM_NOT_ACTIVE if HV is not over threshold
    hv_over_threshold = false;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into WANTING_READY_TO_DRIVE
    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, false);
    start_btn = false;
    brake_pressed = false;

    // Goes into DRIVETRAIN_READY
    drivetrain_ready = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, true);

}

TEST (VehicleStateMachine, ExitsReadyToDriveWhenNoHV) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into WANTING_READY_TO_DRIVE
    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, false);

    // Goes into READY_TO_DRIVE
    drivetrain_ready = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, true);

    // Leaves RTD when HV dips below threshold
    hv_over_threshold = false;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

}

TEST (VehicleStateMachine, ExitsReadyToDriveWhenDrivetrainError) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into WANTING_READY_TO_DRIVE
    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, false);
    start_btn = false;
    brake_pressed = false;

    // Goes into READY_TO_DRIVE
    drivetrain_ready = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, true);

    // Leaves RTD when Drivetrain Error occurs
    drivetrain_error = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

}

// TODO: Add ExitsReadyToDriveWhenSteeringTimeout

TEST (VehicleStateMachine, ExitsReadyToDriveWhenPedalsTimeout) {
    VehicleStateMachine state_machine = VehicleStateMachine(
        mock_hv_over_threshold,
        mock_start_btn,
        mock_brake_pressed,
        mock_drivetrain_error,
        mock_drivetrain_ready,
        mock_start_buzzer,
        mock_recalibrate_pedals,
        mock_command_drivetrain,
        mock_pedals_timeout,
        mock_pedals_reset,
        mock_inverter_button_pressed,
        mock_calibrate_pedals_pressed,
        mock_reset_inverter_error,
        mock_recalibrate_steering,
        mock_is_calibrate_steering_pressed,
        mock_steering_timeout,
        mock_steering_reset
    );
    reset_all_booleans();

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into TRACTIVE_SYSTEM_ACTIVE
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

    // Goes into WANTING_READY_TO_DRIVE
    start_btn = true;
    brake_pressed = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::WANTING_READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, false);
    start_btn = false;
    brake_pressed = false;

    // Goes into READY_TO_DRIVE
    drivetrain_ready = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    ASSERT_DRIVETRAIN_COMMANDED(true, true);

    // Leaves RTD when Pedal Timeout occurs
    pedals_timeout = true;
    state_machine.tick_state_machine(0); // Switches states
    state_machine.tick_state_machine(0); // Evaluates in new state
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    ASSERT_DRIVETRAIN_COMMANDED(false, false);

}