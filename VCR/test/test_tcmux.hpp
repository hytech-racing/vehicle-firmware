#ifndef TEST_TCMUX_HPP
#define TEST_TCMUX_HPP

#include "SystemTimeInterface.h"
#include "TorqueControllerMux.hpp"

#include <gtest/gtest.h>

using ControllerEval = std::function<DrivetrainCommand_s(const VCRData_s& state, unsigned long curr_millis)>;

/**
 * @brief TorqueControllerMux's constructor is private and only VCRControls is a friend. The real VCRControls (VCR/include/controls.hpp)
 *        is not part of the test build, so this stand-in is the friend here and only exists to construct muxes for the tests.
 */
class VCRControls
{
  public:
    template <size_t N>
    static TorqueControllerMux<N> makeMux(std::array<ControllerEval, N> evals, std::array<bool, N> bypass, TCMuxParams_s params)
    {
        return TorqueControllerMux<N>(evals, bypass, params);
    }
};

namespace tcmux_test
{
    // Loose limits so a test only sees the limit it is about
    constexpr TCMuxParams_s LOOSE_PARAMS = {
        .max_speed_during_mode_change = tc_mux_default_params::MAX_SPEED_DURING_MODE_CHANGE,
        .max_torque_delta_during_mode_change = tc_mux_default_params::MAX_TORQUE_DELTA_DURING_MODE_CHANGE,
        .max_power_limit_watts = 1.0e9f,
        .max_avg_torque_nm = 1000.0f,
        .max_avg_speed_rpm = 1.0e6f,
        .num_motors = 4};

    constexpr float REGEN_MIDPOINT_KPH = (tc_mux_default_params::REGEN_ZERO_AT_OR_BELOW_KPH + tc_mux_default_params::REGEN_FULL_AT_OR_ABOVE_KPH) / 2.0f;

    float kphToRPM(float kph)
    {
        return kph / physical_motor_scales::RPM_TO_KILOMETERS_PER_HOUR;
    }

    ControllerEval constantTorque(veh_vec<torque_nm> torques)
    {
        return [torques](const VCRData_s&, unsigned long) -> DrivetrainCommand_s
        {
            return {.control_mode = DrivetrainControlMode_e::TORQUE, .desired_torques = torques, .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};
        };
    }

    ControllerEval constantTorque(float torque)
    {
        return constantTorque({torque, torque, torque, torque});
    }

    ControllerEval constantSpeed(float speed_rpm)
    {
        return [speed_rpm](const VCRData_s&, unsigned long) -> DrivetrainCommand_s
        {
            return {
                .control_mode = DrivetrainControlMode_e::SPEED,
                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                .desired_speeds = {speed_rpm, speed_rpm, speed_rpm, speed_rpm}};
        };
    }

    VCRData_s makeVCRData(veh_vec<speed_rpm> speeds_rpm, volt bus_voltage = 400.0f)
    {
        VCRData_s state = {};
        state.system_data.drivetrain_data.measured_speeds = speeds_rpm;
        state.system_data.drivetrain_data.measured_hv_bus_voltage = {bus_voltage, bus_voltage, bus_voltage, bus_voltage};
        return state;
    }

    VCRData_s makeVCRData(float speed_rpm, volt bus_voltage = 400.0f)
    {
        return makeVCRData({speed_rpm, speed_rpm, speed_rpm, speed_rpm}, bus_voltage);
    }

    /// Single limited controller in MODE_0, the common case for limit tests
    TorqueControllerMux<1> singleControllerMux(ControllerEval eval, TCMuxParams_s params = LOOSE_PARAMS)
    {
        return VCRControls::makeMux<1>({eval}, {false}, params);
    }
} // namespace tcmux_test

using namespace tcmux_test;

/* ---------- Mode selection ---------- */

TEST(TorqueControllerMuxTest, UnsupportedDialModeReturnsZeroAndFlagsError)
{
    auto mux = VCRControls::makeMux<5>(
        {constantTorque(5.0f), constantTorque(5.0f), constantTorque(5.0f), constantTorque(5.0f), constantTorque(5.0f)},
        {false, false, false, false, false},
        LOOSE_PARAMS);

    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_2, makeVCRData(0.0f));
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS);
    for (float t : out.desired_torques.as_array())
    {
        EXPECT_FLOAT_EQ(t, 0.0f);
    }
}

TEST(TorqueControllerMuxTest, ModeBeyondSmallMuxIsRejectedNotIndexedOutOfBounds)
{
    auto mux = singleControllerMux(constantTorque(5.0f));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_4, makeVCRData(0.0f));
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 0.0f);
}

TEST(TorqueControllerMuxTest, UnboundControllerReturnsZeroAndFlagsError)
{
    auto mux = VCRControls::makeMux<2>({constantTorque(5.0f), ControllerEval{}}, {false, false}, LOOSE_PARAMS);
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_1, makeVCRData(0.0f));
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 0.0f);
}

TEST(TorqueControllerMuxTest, ActiveControllerOutputIsPassedThrough)
{
    auto mux = singleControllerMux(constantTorque({1.0f, 2.0f, 3.0f, 4.0f}));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(0.0f));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 1.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.FR, 2.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RL, 3.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RR, 4.0f);
}

/* ---------- Switching controllers ---------- */

TEST(TorqueControllerMuxTest, SwitchesWhenSlowAndTorqueClose)
{
    auto mux = VCRControls::makeMux<2>({constantTorque(1.0f), constantTorque(1.2f)}, {false, false}, LOOSE_PARAMS);
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_1, makeVCRData(0.0f));
    EXPECT_EQ(mux.getTCMuxStatus().active_controller_mode, ControllerMode_e::MODE_1);
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::NO_ERROR);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 1.2f);
}

TEST(TorqueControllerMuxTest, DoesNotSwitchWhileMovingTooFast)
{
    auto mux = VCRControls::makeMux<2>({constantTorque(1.0f), constantTorque(1.0f)}, {false, false}, LOOSE_PARAMS);
    float too_fast_rpm = 2.0f * tc_mux_default_params::MAX_SPEED_DURING_MODE_CHANGE * physical_motor_scales::METERS_PER_SECOND_TO_RPM;

    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_1, makeVCRData(too_fast_rpm));
    EXPECT_EQ(mux.getTCMuxStatus().active_controller_mode, ControllerMode_e::MODE_0);
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 1.0f);
}

TEST(TorqueControllerMuxTest, DoesNotSwitchWhenTorqueJumpIsTooLarge)
{
    auto mux = VCRControls::makeMux<2>({constantTorque(0.0f), constantTorque(5.0f)}, {false, false}, LOOSE_PARAMS);
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_1, makeVCRData(0.0f));
    EXPECT_EQ(mux.getTCMuxStatus().active_controller_mode, ControllerMode_e::MODE_0);
    EXPECT_EQ(mux.getTCMuxStatus().active_error, TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 0.0f);
}

/* ---------- Bypass ---------- */

TEST(TorqueControllerMuxTest, BypassedControllerIsNotLimited)
{
    // Regen at standstill would be zeroed by the mux, a bypassed controller (drivebrain) passes it straight through.
    // The inverter-level brake current cutoff is what protects against this case.
    auto mux = VCRControls::makeMux<1>({constantTorque(-5.0f)}, {true}, LOOSE_PARAMS);
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(0.0f));
    EXPECT_TRUE(mux.getTCMuxStatus().output_is_bypassing_limits);
    EXPECT_FLOAT_EQ(out.desired_torques.FL, -5.0f);
}

TEST(TorqueControllerMuxTest, LimitedControllerReportsNotBypassing)
{
    auto mux = singleControllerMux(constantTorque(1.0f));
    mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(0.0f));
    EXPECT_FALSE(mux.getTCMuxStatus().output_is_bypassing_limits);
}

/* ---------- Torque and power limits ---------- */

TEST(TorqueControllerMuxTest, AverageTorqueLimitScalesAllWheelsKeepingRatios)
{
    TCMuxParams_s params = LOOSE_PARAMS;
    params.max_avg_torque_nm = 2.0f;
    auto mux = singleControllerMux(constantTorque({2.0f, 2.0f, 6.0f, 6.0f}), params); // average 4 Nm

    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(0.0f));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 1.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.FR, 1.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RL, 3.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RR, 3.0f);
}

TEST(TorqueControllerMuxTest, TorqueUnderAverageLimitIsUnchanged)
{
    TCMuxParams_s params = LOOSE_PARAMS;
    params.max_avg_torque_nm = 10.0f;
    auto mux = singleControllerMux(constantTorque(5.0f), params);
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(0.0f));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 5.0f);
}

TEST(TorqueControllerMuxTest, PowerLimitCapsTotalMechanicalPower)
{
    TCMuxParams_s params = LOOSE_PARAMS;
    params.max_power_limit_watts = 1000.0f;
    auto mux = singleControllerMux(constantTorque(10.0f), params);

    float rpm = 1000.0f;
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(rpm));

    float omega = rpm * physical_motor_scales::RPM_TO_RAD_PER_SECOND;
    float total_power = 0.0f;
    for (float t : out.desired_torques.as_array())
    {
        total_power += t * omega;
    }
    EXPECT_NEAR(total_power, 1000.0f, 1.0f);
}

/* ---------- Regen derating (torque mode) ---------- */

TEST(TorqueControllerMuxTest, FullRegenAboveDerateSpeed)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(12.0f)));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, -5.0f);
}

TEST(TorqueControllerMuxTest, NoRegenAtOrBelowRulesSpeed)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    for (float kph : {0.0f, 4.0f, tc_mux_default_params::REGEN_ZERO_AT_OR_BELOW_KPH})
    {
        DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(kph)));
        for (float t : out.desired_torques.as_array())
        {
            EXPECT_FLOAT_EQ(t, 0.0f) << "at " << kph << " km/h";
        }
    }
}

TEST(TorqueControllerMuxTest, RegenRampsLinearlyBetweenZeroAndFullSpeeds)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(REGEN_MIDPOINT_KPH)));
    EXPECT_NEAR(out.desired_torques.FL, -2.5f, 1e-3f);
}

TEST(TorqueControllerMuxTest, RegenIsDeratedPerWheel)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData({kphToRPM(12.0f), kphToRPM(12.0f), kphToRPM(12.0f), kphToRPM(3.0f)}));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, -5.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RR, 0.0f); // stopped wheel gets no regen even though the others are moving
}

TEST(TorqueControllerMuxTest, RegenDerateUsesWheelSpeedMagnitude)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(-kphToRPM(12.0f)));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, -5.0f);
}

TEST(TorqueControllerMuxTest, PositiveTorqueIsNeverRegenDerated)
{
    auto mux = singleControllerMux(constantTorque({5.0f, -5.0f, 5.0f, -5.0f}));
    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(3.0f)));
    EXPECT_FLOAT_EQ(out.desired_torques.FL, 5.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.FR, 0.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RL, 5.0f);
    EXPECT_FLOAT_EQ(out.desired_torques.RR, 0.0f);
}

TEST(TorqueControllerMuxTest, RegenDeratedByBusVoltage)
{
    auto mux = singleControllerMux(constantTorque(-5.0f));
    float mid_voltage = (tc_mux_default_params::REGEN_VOLTAGE_DERATE_START_V + tc_mux_default_params::REGEN_VOLTAGE_CUTOFF_V) / 2.0f;

    DrivetrainCommand_s half = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(12.0f), mid_voltage));
    EXPECT_NEAR(half.desired_torques.FL, -2.5f, 1e-3f);

    DrivetrainCommand_s none = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(12.0f), tc_mux_default_params::REGEN_VOLTAGE_CUTOFF_V));
    EXPECT_FLOAT_EQ(none.desired_torques.FL, 0.0f);
}

TEST(TorqueControllerMuxTest, RegenUsesTheStricterOfSpeedAndVoltageDerating)
{
    auto mux = singleControllerMux(constantTorque(-8.0f));
    float quarter_voltage = tc_mux_default_params::REGEN_VOLTAGE_DERATE_START_V
                            + 0.75f * (tc_mux_default_params::REGEN_VOLTAGE_CUTOFF_V - tc_mux_default_params::REGEN_VOLTAGE_DERATE_START_V); // 0.25

    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(kphToRPM(REGEN_MIDPOINT_KPH), quarter_voltage)); // speed 0.5
    EXPECT_NEAR(out.desired_torques.FL, -2.0f, 1e-3f); // min(0.5, 0.25) * -8, not 0.5 * 0.25 * -8
}

/* ---------- Regen derating (speed mode) ---------- */

TEST(TorqueControllerMuxTest, SpeedModeZeroTargetIsPulledTowardMeasuredSpeedWhenDerated)
{
    auto mux = singleControllerMux(constantSpeed(0.0f));
    float rpm = kphToRPM(REGEN_MIDPOINT_KPH);

    DrivetrainCommand_s out = mux.evaluateTCMux(ControllerMode_e::MODE_0, makeVCRData(rpm));
    EXPECT_NEAR(out.desired_speeds.FL, rpm / 2.0f, 1.0f); // half way between measured speed and the 0 target
}

#endif // TEST_TCMUX_HPP