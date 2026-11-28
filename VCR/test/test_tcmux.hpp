#ifndef __TEST_TCMUX__
#define __TEST_TCMUX__
#include "SharedFirmwareTypes.h"
#include "SystemTimeInterface.h"
#include "TorqueControllerMux.hpp"
#include "controllers/DrivebrainController.hpp"
#include "controllers/SimpleController.hpp"

#include "gtest/gtest.h"
#include <functional>
// TODO
// - [x] test to ensure that the size checking for desired modes works and failes properly
template <typename quad_array_type>
void setFourOutputs(quad_array_type& out, float val)
{
    out.fl = val;
    out.fr = val;
    out.rl = val;
    out.rr = val;
}

template <typename cmd_type>
void setOutputsRpm(cmd_type& cmd, float rpm, float torque)
{
    setFourOutputs(cmd.desired_speeds, rpm);
    setFourOutputs(cmd.torque_limits, torque);
}

template <typename cmd_type>
void setOutputs(cmd_type& cmd, float mps, float torque)
{
    setOutputsRpm(cmd, METERS_PER_SECOND_TO_RPM * mps, torque);
}


TEST(TorqueControllerMuxTesting, test_construction)
{
    auto test_func = [](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return {};
    };
    TorqueControllerMux<2> test({test_func, test_func}, {false, false});
}

TEST(TorqueControllerMuxTesting, test_invalid_controller_request_error)
{

    auto test_func = [](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return {};
    };
    TorqueControllerMux<2> test({test_func, nullptr}, {false, false});
    auto res = test.getDrivetrainCommand(ControllerMode_e::MODE_2, TorqueLimit_e::TCMUX_FULL_TORQUE, {});

    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS);
    for (int i = 0; i < 4; i++)
    {

        ASSERT_EQ(res.desired_speeds.asArray()[i], 0.0);
        ASSERT_EQ(res.torque_limits.asArray()[i], 0.0);
    }

    res = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, {});

    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER);
    for (int i = 0; i < 4; i++)
    {

        ASSERT_EQ(res.desired_speeds.asArray()[i], 0.0);
        ASSERT_EQ(res.torque_limits.asArray()[i], 0.0);
    }
}

// ensure that swapping to a controller that has a higher desired output speed than previously
// commanded that we dont switch
TEST(TorqueControllerMuxTesting, test_controller_output_swap_logic)
{
    DrivetrainCommand_s out1;
    setOutputs(out1, 0, 1);
    DrivetrainCommand_s out2;
    setOutputs(out2, 6, 1);
    auto test_func_1 = [&out1](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return out1;
    };
    auto test_func_2 = [&out2](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return out2;
    };

    TorqueControllerMux<2> test({test_func_1, test_func_2}, {false, false});
    VCRData_s state;
    setFourOutputs(state.system_data.drivetrain_data.measured_speeds, 10000.0);

    auto res1 = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    res1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.getTcMuxStatus().active_controller_mode, ControllerMode_e::MODE_0);
    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH);

    setOutputs(out1, 0, 1);
    setOutputs(out2, 0, 1);
    setFourOutputs(state.system_data.drivetrain_data.measured_speeds, 0);

    res1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.getTcMuxStatus().active_controller_mode, ControllerMode_e::MODE_1);
    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::NO_ERROR);
}

TEST(TorqueControllerMuxTesting, test_torque_diff_swap_limit)
{
    DrivetrainCommand_s inst1, inst2;
    auto test_func_1 = [&inst1](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return inst1;
    };
    auto test_func_2 = [&inst2](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return inst2;
    };
    setOutputs(inst1, 0.1, 1);
    setOutputs(inst2, 3, 10);
    TorqueControllerMux<2> test({test_func_1, test_func_2}, {false, false});
    VCRData_s state;

    state.system_data.drivetrain_data = {};

    auto out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    ASSERT_EQ(test.getTcMuxStatus().active_controller_mode, ControllerMode_e::MODE_0);
    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH);

    // tick it a bunch of times
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH);

    ASSERT_EQ(test.getTcMuxStatus().active_controller_mode, ControllerMode_e::MODE_0);

    ASSERT_EQ(out1.torque_limits.fl, 1);
    ASSERT_EQ(out1.torque_limits.fr, 1);
    ASSERT_EQ(out1.torque_limits.rl, 1);
    ASSERT_EQ(out1.torque_limits.rr, 1);
}

// TEST(TorqueControllerMuxTesting, test_construction_with_new_controller_orgs)
// {
//     // mode 0
//     TorqueControllerSimple tc_simple(1.0f, 1.0f);
//     // mode 1
//     TorqueControllerLoadCellVectoring tc_vec;
//     // mode 2
//     DummyQueue_s q;
//     CASESystem<DummyQueue_s> case_sys(&q, 100, 70, 550, {});
//     TorqueControllerCASEWrapper<DummyQueue_s> case_wrapper(&case_sys);

//     // mode 3
//     TorqueControllerSimpleLaunch simple_launch;
//     // mode 4
//     TorqueControllerSlipLaunch slip_launch;

//     TorqueControllerMux<5>
//         torque_controller_mux({static_cast<Controller *>(&tc_simple),
//                                static_cast<Controller *>(&tc_vec),
//                                static_cast<Controller *>(&case_wrapper),
//                                static_cast<Controller *>(&simple_launch),
//                                static_cast<Controller *>(&slip_launch)},
//                               {false, false, true, false, false});
// }

TEST(TorqueControllerMuxTesting, test_mode0_evaluation)
{

    // mode 0
    TorqueControllerSimpleParams_s standard_params;
    float max_torque = standard_params.amk_max_torque;
    TorqueControllerSimple tc_simple(standard_params);
    //     // mode 1
    //     TorqueControllerLoadCellVectoring tc_vec;
    //     // mode 2
    //     DummyQueue_s q;
    //     CASESystem<DummyQueue_s> case_sys(&q, 100, 70, 550, {});
    //     TorqueControllerCASEWrapper<DummyQueue_s> case_wrapper(&case_sys);

    //     // mode 3
    //     TorqueControllerSimpleLaunch simple_launch;
    //     // mode 4
    //     TorqueControllerSlipLaunch slip_launch;
    //     TorqueControllerMux<5> torque_controller_mux({static_cast<Controller *>(&tc_simple),
    //                                                   static_cast<Controller *>(&tc_vec),
    //                                                   static_cast<Controller *>(&case_wrapper),
    //                                                   static_cast<Controller *>(&simple_launch),
    //                                                   static_cast<Controller *>(&slip_launch)},
    //                                                  {false, true, false, false, false});
    TorqueControllerMux<1> torque_controller_mux(
        {std::bind(&TorqueControllerSimple::evaluate, std::ref(tc_simple), std::placeholders::_1, std::placeholders::_2)},
        {false});

    VCRData_s mode_0_input_state;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.accel_percent = 0.5f;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.brake_percent = 0.0f;

    DrivetrainCommand_s out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    ASSERT_NEAR(out.torque_limits.fl, (max_torque / 2) * (2 - standard_params.rear_torque_scale), 0.01);
    ASSERT_NEAR(out.torque_limits.fr, (max_torque / 2) * (2 - standard_params.rear_torque_scale), 0.01);
    ASSERT_NEAR(out.torque_limits.rl, (max_torque / 2) * standard_params.rear_torque_scale, 0.01);
    ASSERT_NEAR(out.torque_limits.rr, (max_torque / 2) * standard_params.rear_torque_scale, 0.01);

    //     mode_0_input_state = {{}, {}, {}, {}, {.accelPercent = 0.0f, .brakePercent = 0.0f,
    //     .regenPercent = 0.0}, {}, {}, {}}; out =
    //     torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0,
    //     TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state); ASSERT_EQ(out.torque_limits.FL,
    //     0); ASSERT_EQ(out.torque_limits.FR, 0); ASSERT_EQ(out.torque_limits.FR, 0);
    //     ASSERT_EQ(out.torque_limits.FR, 0);

    // out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1,
    // TorqueLimit_e::TCMUX_FULL_TORQUE, mode_1_input_state);
}

TEST(TorqueControllerMuxTesting, test_power_limit)
{
    // mode 0
    TorqueControllerSimpleParams_s standard_params;
    float max_torque = standard_params.amk_max_torque;
    TorqueControllerSimple tc_simple(standard_params);

    DrivetrainDynamicReport_s drivetrain_data = {};
    setFourOutputs(drivetrain_data.measured_speeds, 1000.0f);

    VCRData_s mode_0_input_state;
    mode_0_input_state.system_data.drivetrain_data = drivetrain_data;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.accel_percent = 0.5f;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.brake_percent = 0.0f;

    float new_power_lim = 20000.0f;
    TorqueControllerMux<1> torque_controller_mux(
        {std::bind(&TorqueControllerSimple::evaluate, std::ref(tc_simple), std::placeholders::_1, std::placeholders::_2)},
        {false},
        TC_MUX_DEFAULT_PARAMS::MAX_SPEED_FOR_MODE_CHANGE,
        TC_MUX_DEFAULT_PARAMS::MAX_TORQUE_DELTA_FOR_MODE_CHANGE,
        new_power_lim);

    DrivetrainCommand_s out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    ASSERT_NEAR(out.torque_limits.fl, (max_torque / 2) * (2 - standard_params.rear_torque_scale), 0.01);
    ASSERT_NEAR(out.torque_limits.fr, (max_torque / 2) * (2 - standard_params.rear_torque_scale), 0.01);
    ASSERT_NEAR(out.torque_limits.rl, (max_torque / 2) * standard_params.rear_torque_scale, 0.01);
    ASSERT_NEAR(out.torque_limits.rr, (max_torque / 2) * standard_params.rear_torque_scale, 0.01);


    float rpm_set = 20000.0f;
    setFourOutputs(drivetrain_data.measured_speeds, rpm_set);
    mode_0_input_state.system_data.drivetrain_data = drivetrain_data;
    out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);

    ASSERT_NEAR(
        ((out.torque_limits.fl * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.fr * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.rl * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.rr * (rpm_set * RPM_TO_RAD_PER_SECOND))),
        new_power_lim,
        0.1);
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.accel_percent = 0.6f;
    setFourOutputs(drivetrain_data.measured_speeds, rpm_set);
    mode_0_input_state.system_data.drivetrain_data = drivetrain_data;
    (void)torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    (void)torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    (void)torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    (void)torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
    ASSERT_NEAR(
        ((out.torque_limits.fl * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.fr * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.rl * (rpm_set * RPM_TO_RAD_PER_SECOND))
         + (out.torque_limits.rr * (rpm_set * RPM_TO_RAD_PER_SECOND))),
        new_power_lim,
        0.1);
}

TEST(TorqueControllerMuxTesting, test_torque_limit)
{

    // TestControllerType inst1;
    DrivetrainCommand_s inst1;
    auto test_func_1 = [&inst1](const VCRData_s& state, unsigned long m) -> DrivetrainCommand_s
    {
        return inst1;
    };
    setOutputs(inst1, 500, 10.0);
    inst1.torque_limits.fl = 5;
    TorqueControllerMux<1> test({test_func_1}, {false});

    DrivetrainDynamicReport_s drivetrain_data = {};
    setFourOutputs(drivetrain_data.measured_speeds, 500.0f);

    VCRData_s mode_0_input_state;
    mode_0_input_state.system_data.drivetrain_data = drivetrain_data;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.accel_percent = 0.5f;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.brake_percent = 0.0f;
    mode_0_input_state.interface_data.recvd_pedals_data.pedals_data.regen_percent = 0.0f;
    auto drive_command = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, mode_0_input_state);

    ASSERT_EQ(drive_command.torque_limits.fl, 5.0f);
    ASSERT_EQ(drive_command.torque_limits.fr, 10.0f);
    ASSERT_EQ(drive_command.torque_limits.rl, 10.0f);
    ASSERT_EQ(drive_command.torque_limits.rr, 10.0f);

    setOutputs(inst1, 500, 20.0);
    inst1.torque_limits.fl = 5;

    drive_command = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, mode_0_input_state);

    ASSERT_LT(drive_command.torque_limits.fl, 3.5f);
    ASSERT_LT(drive_command.torque_limits.fr, 12.5f);
    ASSERT_LT(drive_command.torque_limits.rl, 12.5f);
    ASSERT_LT(drive_command.torque_limits.rr, 12.5f);

    printf("torque 1: %.2f\n", drive_command.torque_limits.fl);
    printf("torque 2: %.2f\n", drive_command.torque_limits.fr);
    printf("torque 3: %.2f\n", drive_command.torque_limits.rl);
    printf("torque 4: %.2f\n", drive_command.torque_limits.rr);
}

TEST(TorqueControllerMuxTesting, test_null_pointer_error_state)
{
    TorqueControllerMux<1> test({nullptr}, {true});
    VCRData_s state;
    auto res = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, state);
    for (int i = 0; i < 4; i++)
    {
        ASSERT_EQ(res.desired_speeds.asArray()[i], 0.0f);
        ASSERT_EQ(res.torque_limits.asArray()[i], 0.0f);
    }
    ASSERT_EQ(test.getTcMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER);
}

TEST(TorqueControllerMuxTesting, test_drivebrain_and_simple_controller_integration)
{
    TorqueControllerSimple simple_controller;
    DrivebrainController db_controller(10);

    StampedDrivetrainCommand_s data;
    data.desired_speeds.last_recv_millis = 1000;
    data.desired_speeds.recvd = true;
    data.torque_limits.last_recv_millis = 1000;
    sys_time::set_millis(1000);
    data.torque_limits.recvd = true;
    data.torque_limits.veh_vec_data = {1, 1, 1, 1};
    data.desired_speeds.veh_vec_data = {1, 1, 1, 1};

    // TorqueControllerMuxStatus_s status = {};
    // status.active_controller_mode = current_control_mode;
    PedalsSystemData_s pedals_data = {};
    pedals_data.brake_percent = 0.20;
    pedals_data.accel_percent = 0.0;

    VCRData_s state;
    state.interface_data.latest_drivebrain_telem_command = data;
    state.interface_data.recvd_pedals_data.pedals_data = pedals_data;

    // VCRData_s state;


    TorqueControllerMux<2> torque_controller_mux(
        {std::bind(&TorqueControllerSimple::evaluate, std::ref(simple_controller), std::placeholders::_1, std::placeholders::_2),
         std::bind(&DrivebrainController::evaluate, std::ref(db_controller), std::placeholders::_1, std::placeholders::_2)},
        {false, true});

    auto res = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    res = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    EXPECT_FALSE(db_controller.getTimingFailureStatus());
    EXPECT_FLOAT_EQ(res.desired_speeds.fl, data.desired_speeds.veh_vec_data.fl);

    data.desired_speeds.last_recv_millis = 3000;
    data.torque_limits.last_recv_millis = 2000;
    sys_time::set_millis(3000);
    state.interface_data.latest_drivebrain_telem_command = data;
    res = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    EXPECT_TRUE(db_controller.getTimingFailureStatus());


    state.interface_data.dash_input_state.mc_reset_btn_is_pressed = true;
    res = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    data.torque_limits.last_recv_millis = 3000;
    state.interface_data.latest_drivebrain_telem_command = data;
    res = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    EXPECT_FALSE(db_controller.getTimingFailureStatus());
}

#endif // __TEST_TCMUX__
