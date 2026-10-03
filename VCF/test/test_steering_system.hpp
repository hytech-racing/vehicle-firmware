#define STEERING_SYSTEM_TEST
#include <gtest/gtest.h>

#include <cstdint>
#include <iostream>
#include "SteeringSystem.hpp"


/* Timing: the steering task runs at 1 kHz; the filter and rates update every other call (~500 Hz) */
constexpr uint32_t TASK_PERIOD_US   = 1000;
constexpr uint32_t SAMPLE_PERIOD_US = 2000;
constexpr float SAMPLE_PERIOD_SEC   = 0.002f;

/* Analog sensor: 12-bit, 360 deg over 4096 counts */
constexpr uint32_t ANALOG_MIN_RAW = 1024;
constexpr uint32_t ANALOG_MAX_RAW = 3072;
constexpr float ANALOG_DEG_PER_COUNT = 360.0f / 4096.0f;   // 0.087890625

/* Digital sensor: 14-bit, 360 deg over 16384 counts, mounted reversed */
constexpr uint32_t DIGITAL_MIN_RAW = 1000;
constexpr uint32_t DIGITAL_MAX_RAW = 9000;
constexpr float DIGITAL_DEG_PER_COUNT = 360.0f / 16384.0f;  // 0.02197265625 (exactly 1/4 of analog)

constexpr float OOR_TOLERANCE_FRACTION = 0.005f;
constexpr float MAX_STEERING_RATE_DEG_PER_S = 1000.0f;
constexpr float MAX_SENSOR_DISAGREEMENT_DEG = 5.0f;

/* Expected derived values for the default params */
constexpr uint32_t ANALOG_CENTER_RAW = 2048;
constexpr uint32_t DIGITAL_CENTER_RAW = 5000;
constexpr int32_t ANALOG_MIN_VALID_RAW = 1013;  // 1024 - (2048 * 0.005 = 10.24), truncated
constexpr int32_t ANALOG_MAX_VALID_RAW = 3082;  // 3072 + 10.24, truncated
constexpr int32_t DIGITAL_MIN_VALID_RAW = 960;  // 1000 - (8000 * 0.005 = 40)
constexpr int32_t DIGITAL_MAX_VALID_RAW = 9040; // 9000 + 40

/* Same physical angle on both sensors: +100 analog counts == -400 digital counts == +8.7890625 deg */
constexpr uint32_t ANALOG_RAW_AT_8_79_DEG  = 2148;
constexpr uint32_t DIGITAL_RAW_AT_8_79_DEG = 4600;
constexpr float ANGLE_8_79_DEG             = 8.7890625f;

/* Mirrors SteeringSystem::ANALOG_RAW_LIMITS / DIGITAL_RAW_LIMITS (private). Update if those change. */
constexpr uint32_t ANALOG_MIN_UNCLIPPED_RAW  = 5;
constexpr uint32_t ANALOG_MAX_UNCLIPPED_RAW  = 3675;
constexpr uint32_t DIGITAL_MIN_UNCLIPPED_RAW = 10;
constexpr uint32_t DIGITAL_MAX_UNCLIPPED_RAW = 16374;

/* A raw value below every sensor's unclipped range: fed to one sensor to leave its extremes untouched */
constexpr uint32_t CLIPPED_RAW = 0;

/**
 * @brief Builds one sensor's params the same way initializeAllSystems() / _recalibrateSensor() do, from a calibrated min/max
*/
SteeringSensorParams_s make_sensor_params(const uint32_t min_raw, const uint32_t max_raw, const float deg_per_count, const float oor_tolerance_fraction)
{
    SteeringSensorParams_s sensor {};
    sensor.deg_per_count = deg_per_count;
    sensor.oor_tolerance_fraction = oor_tolerance_fraction;

    SteeringSensorCalibration_s &calibration_snapshot = sensor.calibration_snapshot;
    calibration_snapshot.min_raw    = min_raw;
    calibration_snapshot.max_raw    = max_raw;
    calibration_snapshot.span_raw   = max_raw - min_raw;
    calibration_snapshot.center_raw = min_raw + (calibration_snapshot.span_raw / 2);

    sensor.oor_tolerance_raw = static_cast<float>(calibration_snapshot.span_raw) * oor_tolerance_fraction;
    sensor.oor_bounds.min_valid_raw = static_cast<int32_t>(static_cast<float>(min_raw) - sensor.oor_tolerance_raw);
    sensor.oor_bounds.max_valid_raw = static_cast<int32_t>(static_cast<float>(max_raw) + sensor.oor_tolerance_raw);
    return sensor;
}

SteeringSystemParams_s gen_default_params()
{
    SteeringSystemParams_s params {};
    params.analog  = make_sensor_params(ANALOG_MIN_RAW, ANALOG_MAX_RAW, ANALOG_DEG_PER_COUNT, OOR_TOLERANCE_FRACTION);
    params.digital = make_sensor_params(DIGITAL_MIN_RAW, DIGITAL_MAX_RAW, DIGITAL_DEG_PER_COUNT, OOR_TOLERANCE_FRACTION);
    params.max_steering_rate_deg_per_s = MAX_STEERING_RATE_DEG_PER_S;
    params.max_sensor_disagreement_deg = MAX_SENSOR_DISAGREEMENT_DEG;
    return params;
}

static SteeringEncoderReading_s hardcode_digital_data(const uint32_t raw_value, const SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL)
{
    SteeringEncoderReading_s data {};
    data.rawValue = raw_value;
    data.status = status;
    data.angle = 0.0f; // calculated by the system, not the sensor
    data.errors = {};  // no error flags set
    return data;
}

float expected_analog_deg(const uint32_t analog_raw)
{
    return static_cast<float>(static_cast<int32_t>(analog_raw) - static_cast<int32_t>(ANALOG_CENTER_RAW)) * ANALOG_DEG_PER_COUNT;
}

float expected_digital_deg(const uint32_t digital_raw)
{
    // Digital sensor is reversed: raw counts decrease as steering angle increases
    return static_cast<float>(static_cast<int32_t>(DIGITAL_CENTER_RAW) - static_cast<int32_t>(digital_raw)) * DIGITAL_DEG_PER_COUNT;
}

void debug_print_steering(const SteeringSystemData_s &data)
{
    std::cout << "analog_steering_angle: "              << data.analog_steering_angle << " deg\n";
    std::cout << "digital_steering_angle: "             << data.digital_steering_angle << " deg\n";
    std::cout << "output_steering_angle: "              << data.output_steering_angle << " deg\n";
    std::cout << "analog_steering_velocity_deg_s: "     << data.analog_steering_velocity_deg_s << "\n";
    std::cout << "digital_steering_velocity_deg_s: "    << data.digital_steering_velocity_deg_s << "\n";
    std::cout << "analog_oor_implausibility: "          << data.analog_oor_implausibility << "\n";
    std::cout << "digital_oor_implausibility: "         << data.digital_oor_implausibility << "\n";
    std::cout << "sensor_disagreement_implausibility: " << data.sensor_disagreement_implausibility << "\n";
    std::cout << "dtheta_exceeded_analog: "             << data.dtheta_exceeded_analog << "\n";
    std::cout << "dtheta_exceeded_digital: "            << data.dtheta_exceeded_digital << "\n";
    std::cout << "both_sensors_fail: "                  << data.both_sensors_fail << "\n";
}

class SteeringSystemTest : public ::testing::Test
{
protected:

    SteeringSystemParams_s _params = gen_default_params();
    SteeringSystem _steering {_params};
    uint32_t _last_time_us = 0;

    /** One call of the steering task at an explicit time. */
    void evaluateAt(const uint32_t analog_raw, const uint32_t digital_raw, const uint32_t time_us,
                    const SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL)
    {
        _steering.evaluateSteering(analog_raw, hardcode_digital_data(digital_raw, status), time_us);
        _last_time_us = time_us;
    }

    void evaluateNextSample(const uint32_t analog_raw, const uint32_t digital_raw,
                            const SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL)
    {
        evaluateAt(analog_raw, digital_raw, _last_time_us + SAMPLE_PERIOD_US, status);
    }

    const SteeringSystemData_s &data() const { return _steering.getSteeringSystemData(); }
    const SteeringSystemParams_s &currentParams() const { return _steering.getSteeringParams(); }
};


TEST_F(SteeringSystemTest, default_params_have_expected_calibration_and_bounds)
{
    const SteeringSensorParams_s &analog = _params.analog;
    EXPECT_EQ(analog.calibration_snapshot.span_raw, 2048U);
    EXPECT_EQ(analog.calibration_snapshot.center_raw, ANALOG_CENTER_RAW);
    EXPECT_EQ(analog.oor_bounds.min_valid_raw, ANALOG_MIN_VALID_RAW);
    EXPECT_EQ(analog.oor_bounds.max_valid_raw, ANALOG_MAX_VALID_RAW);

    const SteeringSensorParams_s &digital = _params.digital;
    EXPECT_EQ(digital.calibration_snapshot.span_raw, 8000U);
    EXPECT_EQ(digital.calibration_snapshot.center_raw, DIGITAL_CENTER_RAW);
    EXPECT_EQ(digital.oor_bounds.min_valid_raw, DIGITAL_MIN_VALID_RAW);
    EXPECT_EQ(digital.oor_bounds.max_valid_raw, DIGITAL_MAX_VALID_RAW);
}

/* ============================================================================================
 * Raw -> degree conversion
 * ============================================================================================ */

TEST_F(SteeringSystemTest, center_raw_converts_to_zero_degrees)
{
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, 0);

    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), 0.0f, 0.001f);
    EXPECT_NEAR(data().digital_steering_angle, 0.0f, 0.001f);
}

TEST_F(SteeringSystemTest, analog_min_and_max_convert_to_expected_degrees)
{
    // Unfiltered angle is used so the filter's lag doesn't affect the conversion check
    evaluateNextSample(ANALOG_MIN_RAW, DIGITAL_CENTER_RAW);
    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), -90.0f, 0.001f);

    evaluateNextSample(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW);
    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), 90.0f, 0.001f);
}

TEST_F(SteeringSystemTest, digital_conversion_is_reversed)
{
    // Lower raw count = positive angle
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_MIN_RAW);
    EXPECT_NEAR(data().digital_steering_angle, expected_digital_deg(DIGITAL_MIN_RAW), 0.001f);
    EXPECT_GT(data().digital_steering_angle, 0.0f);

    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_MAX_RAW);
    EXPECT_NEAR(data().digital_steering_angle, expected_digital_deg(DIGITAL_MAX_RAW), 0.001f);
    EXPECT_LT(data().digital_steering_angle, 0.0f);
}

TEST_F(SteeringSystemTest, same_physical_angle_matches_on_both_sensors)
{
    evaluateAt(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG, 0);

    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), ANGLE_8_79_DEG, 0.001f);
    EXPECT_NEAR(data().digital_steering_angle, ANGLE_8_79_DEG, 0.001f);
}

/* ============================================================================================
 * Out-of-range checks
 * ============================================================================================ */

TEST_F(SteeringSystemTest, oor_not_evaluated_on_first_tick)
{
    // The first tick only seeds the filter and rate references, then returns before any checks run
    evaluateAt(ANALOG_MAX_RAW + 1000, DIGITAL_MAX_RAW + 1000, 0);

    EXPECT_FALSE(data().analog_oor_implausibility);
    EXPECT_FALSE(data().digital_oor_implausibility);
}

TEST_F(SteeringSystemTest, analog_oor_bounds_are_inclusive)
{
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);  // seed

    evaluateNextSample(static_cast<uint32_t>(ANALOG_MIN_VALID_RAW), DIGITAL_CENTER_RAW);
    EXPECT_FALSE(data().analog_oor_implausibility);

    evaluateNextSample(static_cast<uint32_t>(ANALOG_MIN_VALID_RAW) - 1, DIGITAL_CENTER_RAW);
    EXPECT_TRUE(data().analog_oor_implausibility);

    evaluateNextSample(static_cast<uint32_t>(ANALOG_MAX_VALID_RAW), DIGITAL_CENTER_RAW);
    EXPECT_FALSE(data().analog_oor_implausibility);

    evaluateNextSample(static_cast<uint32_t>(ANALOG_MAX_VALID_RAW) + 1, DIGITAL_CENTER_RAW);
    EXPECT_TRUE(data().analog_oor_implausibility);
}

TEST_F(SteeringSystemTest, digital_oor_bounds_are_inclusive)
{
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);  // seed

    evaluateNextSample(ANALOG_CENTER_RAW, static_cast<uint32_t>(DIGITAL_MIN_VALID_RAW));
    EXPECT_FALSE(data().digital_oor_implausibility);

    evaluateNextSample(ANALOG_CENTER_RAW, static_cast<uint32_t>(DIGITAL_MIN_VALID_RAW) - 1);
    EXPECT_TRUE(data().digital_oor_implausibility);

    evaluateNextSample(ANALOG_CENTER_RAW, static_cast<uint32_t>(DIGITAL_MAX_VALID_RAW));
    EXPECT_FALSE(data().digital_oor_implausibility);

    evaluateNextSample(ANALOG_CENTER_RAW, static_cast<uint32_t>(DIGITAL_MAX_VALID_RAW) + 1);
    EXPECT_TRUE(data().digital_oor_implausibility);
}

/* ============================================================================================
 * Steering rate (velocity) checks
 * ============================================================================================ */

TEST_F(SteeringSystemTest, no_rate_flags_on_first_tick)
{
    evaluateAt(ANALOG_MAX_RAW, DIGITAL_MIN_RAW, 0);

    EXPECT_FALSE(data().dtheta_exceeded_analog);
    EXPECT_FALSE(data().dtheta_exceeded_digital);
}

TEST_F(SteeringSystemTest, digital_velocity_uses_measured_interval)
{
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, 0);

    // 4 counts in 2000 us
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 4, 2000);
    EXPECT_NEAR(data().digital_steering_velocity_deg_s, (4.0f * DIGITAL_DEG_PER_COUNT) / 0.002f, 0.001f);

    // 4 more counts in 4000 us: same motion over twice the time gives half the velocity
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 8, 6000);
    EXPECT_NEAR(data().digital_steering_velocity_deg_s, (4.0f * DIGITAL_DEG_PER_COUNT) / 0.004f, 0.001f);
}

TEST_F(SteeringSystemTest, digital_rate_check_flags_large_jump_only)
{
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);

    // 1000 counts (~22 deg) in 2 ms is ~11,000 deg/s
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 1000);
    EXPECT_TRUE(data().dtheta_exceeded_digital);

    // 4 counts in 2 ms is ~44 deg/s
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 1004);
    EXPECT_FALSE(data().dtheta_exceeded_digital);
}

TEST_F(SteeringSystemTest, analog_rate_check_sees_filtered_angle)
{
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);

    // Step the analog sensor from 0 to +90 deg. The filter lets only a tiny fraction through
    // on the first update (~106 deg/s), so the rate check does not trip immediately.
    evaluateNextSample(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW);
    EXPECT_FALSE(data().dtheta_exceeded_analog);

    // As the filtered angle catches up, its slope peaks around ~2000 deg/s after ~20 ms
    bool rate_flag_seen = false;
    for (int update = 0; update < 30; ++update)
    {
        evaluateNextSample(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW);
        rate_flag_seen = rate_flag_seen || data().dtheta_exceeded_analog;
    }
    EXPECT_TRUE(rate_flag_seen);
}

TEST_F(SteeringSystemTest, velocity_is_held_between_updates_but_angle_is_not)
{
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, 0);
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 4, 2000);
    const float velocity_at_update = data().digital_steering_velocity_deg_s;

    // 1000 us later: below the gate, so no rate update, but the angle still follows the sensor
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 100, 3000);
    EXPECT_FLOAT_EQ(data().digital_steering_velocity_deg_s, velocity_at_update);
    EXPECT_NEAR(data().digital_steering_angle, expected_digital_deg(DIGITAL_CENTER_RAW - 100), 0.001f);
}

/* ============================================================================================
 * Sample gate / timing
 * ============================================================================================ */

TEST_F(SteeringSystemTest, filter_updates_every_other_task_call)
{
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, 0);  // seed: filtered angle = 0

    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 1 * TASK_PERIOD_US);  // 1000 us: held
    EXPECT_FLOAT_EQ(data().analog_steering_angle, 0.0f);

    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 2 * TASK_PERIOD_US);  // 2000 us: update
    const float angle_after_first_update = data().analog_steering_angle;
    EXPECT_GT(angle_after_first_update, 0.0f);

    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 3 * TASK_PERIOD_US);  // 1000 us since update: held
    EXPECT_FLOAT_EQ(data().analog_steering_angle, angle_after_first_update);

    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 4 * TASK_PERIOD_US);  // 2000 us since update: update
    EXPECT_GT(data().analog_steering_angle, angle_after_first_update);
}

TEST_F(SteeringSystemTest, gate_tolerates_scheduler_jitter)
{
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, 0);

    // Just under the 1500 us gate: held
    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 1400);
    EXPECT_FLOAT_EQ(data().analog_steering_angle, 0.0f);

    // A late-then-early call pair (1600 us instead of 2000 us) still updates
    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 1600);
    const float angle_after_update = data().analog_steering_angle;
    EXPECT_GT(angle_after_update, 0.0f);

    // Next call 1000 us later: held again
    evaluateAt(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW, 2600);
    EXPECT_FLOAT_EQ(data().analog_steering_angle, angle_after_update);
}

TEST_F(SteeringSystemTest, handles_micros_wraparound)
{
    const uint32_t start_us = UINT32_MAX - 499;  // 500 us before micros() wraps to 0
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW, start_us);

    // 1500 after the wrap is 2000 us later
    evaluateAt(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 4, 1500);
    EXPECT_NEAR(data().digital_steering_velocity_deg_s, (4.0f * DIGITAL_DEG_PER_COUNT) / SAMPLE_PERIOD_SEC, 0.001f);
    EXPECT_FALSE(data().dtheta_exceeded_digital);
}

/* ============================================================================================
 * Analog low-pass filter
 * ============================================================================================ */

TEST_F(SteeringSystemTest, filter_seeds_to_first_reading)
{
    // No ramp up from 0: the first filtered angle equals the first reading
    evaluateAt(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG, 0);
    EXPECT_NEAR(data().analog_steering_angle, ANGLE_8_79_DEG, 0.0001f);
}

TEST_F(SteeringSystemTest, filter_holds_steady_input)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    for (int update = 0; update < 100; ++update)
    {
        evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    }
    EXPECT_NEAR(data().analog_steering_angle, ANGLE_8_79_DEG, 0.001f);
    EXPECT_NEAR(data().analog_steering_velocity_deg_s, 0.0f, 0.01f);
}

TEST_F(SteeringSystemTest, filter_smooths_step_input)
{
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);

    // Step from 0 to +90 deg
    evaluateNextSample(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW);
    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), 90.0f, 0.001f);  // unfiltered jumps immediately
    EXPECT_GT(data().analog_steering_angle, 0.0f);
    EXPECT_LT(data().analog_steering_angle, 0.9f);                          // filtered moves < 1% of the step

    // After ~200 ms the filtered angle has settled on the new value
    for (int update = 0; update < 100; ++update)
    {
        evaluateNextSample(ANALOG_MAX_RAW, DIGITAL_CENTER_RAW);
    }
    EXPECT_NEAR(data().analog_steering_angle, 90.0f, 0.5f);
}

/* ============================================================================================
 * Sensor disagreement
 * ============================================================================================ */

TEST_F(SteeringSystemTest, sensors_reading_same_angle_agree)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);

    EXPECT_FALSE(data().sensor_disagreement_implausibility);
}

TEST_F(SteeringSystemTest, disagreement_threshold_boundary)
{
    // Analog at 0 deg throughout
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW);

    // 223 digital counts = 4.90 deg: within the 5 deg tolerance
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 223);
    EXPECT_FALSE(data().sensor_disagreement_implausibility);

    // 228 digital counts = 5.01 deg: beyond the tolerance
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 228);
    EXPECT_TRUE(data().sensor_disagreement_implausibility);
}

/* ============================================================================================
 * Sensor selection
 * Note: the first tick returns before selection runs, so every case needs at least two calls.
 * ============================================================================================ */

TEST_F(SteeringSystemTest, output_uses_digital_when_both_valid)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);

    EXPECT_FLOAT_EQ(data().output_steering_angle, data().digital_steering_angle);
    EXPECT_FALSE(data().both_sensors_fail);
}

TEST_F(SteeringSystemTest, output_uses_digital_when_sensors_disagree)
{
    // TODO: update when disagreement starts influencing sensor selection
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 1000);
    evaluateNextSample(ANALOG_CENTER_RAW, DIGITAL_CENTER_RAW - 1000);

    EXPECT_TRUE(data().sensor_disagreement_implausibility);
    EXPECT_FLOAT_EQ(data().output_steering_angle, data().digital_steering_angle);
}

TEST_F(SteeringSystemTest, output_falls_back_to_analog_when_digital_oor)
{
    const uint32_t digital_oor_raw = static_cast<uint32_t>(DIGITAL_MAX_VALID_RAW) + 500;
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, digital_oor_raw);
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, digital_oor_raw);

    EXPECT_TRUE(data().digital_oor_implausibility);
    EXPECT_FALSE(data().analog_oor_implausibility);
    EXPECT_NEAR(data().output_steering_angle, ANGLE_8_79_DEG, 0.001f);
    EXPECT_FALSE(data().both_sensors_fail);
}

TEST_F(SteeringSystemTest, output_falls_back_to_analog_on_interface_error)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG, SteeringEncoderStatus_e::ERROR);
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG, SteeringEncoderStatus_e::ERROR);

    EXPECT_TRUE(data().interface_sensor_error);
    EXPECT_FLOAT_EQ(data().output_steering_angle, data().analog_steering_angle);
}

TEST_F(SteeringSystemTest, output_falls_back_to_analog_when_digital_rate_exceeded)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);

    // Digital jumps 1000 counts in one sample; analog stays put
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG - 1000);

    EXPECT_TRUE(data().dtheta_exceeded_digital);
    EXPECT_FALSE(data().dtheta_exceeded_analog);
    EXPECT_NEAR(data().output_steering_angle, ANGLE_8_79_DEG, 0.001f);
}

TEST_F(SteeringSystemTest, output_uses_digital_when_analog_oor)
{
    const uint32_t analog_oor_raw = static_cast<uint32_t>(ANALOG_MAX_VALID_RAW) + 400;
    evaluateNextSample(analog_oor_raw, DIGITAL_RAW_AT_8_79_DEG);
    evaluateNextSample(analog_oor_raw, DIGITAL_RAW_AT_8_79_DEG);

    EXPECT_TRUE(data().analog_oor_implausibility);
    EXPECT_FALSE(data().digital_oor_implausibility);
    EXPECT_NEAR(data().output_steering_angle, ANGLE_8_79_DEG, 0.001f);
}

TEST_F(SteeringSystemTest, both_sensors_fail_holds_last_good_output)
{
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    evaluateNextSample(ANALOG_RAW_AT_8_79_DEG, DIGITAL_RAW_AT_8_79_DEG);
    const float last_good_output = data().output_steering_angle;
    EXPECT_NEAR(last_good_output, ANGLE_8_79_DEG, 0.001f);

    // Both sensors go out of range
    evaluateNextSample(static_cast<uint32_t>(ANALOG_MAX_VALID_RAW) + 400, static_cast<uint32_t>(DIGITAL_MAX_VALID_RAW) + 500);

    EXPECT_TRUE(data().analog_oor_implausibility);
    EXPECT_TRUE(data().digital_oor_implausibility);
    EXPECT_TRUE(data().both_sensors_fail);
    EXPECT_FLOAT_EQ(data().output_steering_angle, last_good_output);
}

/* ============================================================================================
 * Observed extremes
 * ============================================================================================ */

TEST_F(SteeringSystemTest, observed_extremes_start_empty)
{
    EXPECT_EQ(currentParams().analog.observed_extremes.min_raw, UINT32_MAX);
    EXPECT_EQ(currentParams().analog.observed_extremes.max_raw, 0U);
    EXPECT_EQ(currentParams().digital.observed_extremes.min_raw, UINT32_MAX);
    EXPECT_EQ(currentParams().digital.observed_extremes.max_raw, 0U);
}

TEST_F(SteeringSystemTest, observed_extremes_track_min_and_max)
{
    _steering.updateObservedExtremes(2000, 5000);
    _steering.updateObservedExtremes(1500, 3000);
    _steering.updateObservedExtremes(2500, 7000);
    _steering.updateObservedExtremes(1800, 6000);  // inside the range: no change

    EXPECT_EQ(currentParams().analog.observed_extremes.min_raw, 1500U);
    EXPECT_EQ(currentParams().analog.observed_extremes.max_raw, 2500U);
    EXPECT_EQ(currentParams().digital.observed_extremes.min_raw, 3000U);
    EXPECT_EQ(currentParams().digital.observed_extremes.max_raw, 7000U);
}

TEST_F(SteeringSystemTest, clipped_analog_readings_are_ignored)
{
    const SteeringSensorObservedExtremes_s &analog = currentParams().analog.observed_extremes;

    _steering.updateObservedExtremes(ANALOG_MIN_UNCLIPPED_RAW - 1, CLIPPED_RAW);
    _steering.updateObservedExtremes(ANALOG_MAX_UNCLIPPED_RAW + 1, CLIPPED_RAW);
    EXPECT_EQ(analog.min_raw, UINT32_MAX);
    EXPECT_EQ(analog.max_raw, 0U);

    // Limits themselves are accepted
    _steering.updateObservedExtremes(ANALOG_MIN_UNCLIPPED_RAW, CLIPPED_RAW);
    _steering.updateObservedExtremes(ANALOG_MAX_UNCLIPPED_RAW, CLIPPED_RAW);
    EXPECT_EQ(analog.min_raw, ANALOG_MIN_UNCLIPPED_RAW);
    EXPECT_EQ(analog.max_raw, ANALOG_MAX_UNCLIPPED_RAW);
}

TEST_F(SteeringSystemTest, clipped_digital_readings_are_ignored)
{
    const SteeringSensorObservedExtremes_s &digital = currentParams().digital.observed_extremes;

    _steering.updateObservedExtremes(CLIPPED_RAW, DIGITAL_MIN_UNCLIPPED_RAW - 1);
    _steering.updateObservedExtremes(CLIPPED_RAW, DIGITAL_MAX_UNCLIPPED_RAW + 1);
    EXPECT_EQ(digital.min_raw, UINT32_MAX);
    EXPECT_EQ(digital.max_raw, 0U);

    _steering.updateObservedExtremes(CLIPPED_RAW, DIGITAL_MIN_UNCLIPPED_RAW);
    _steering.updateObservedExtremes(CLIPPED_RAW, DIGITAL_MAX_UNCLIPPED_RAW);
    EXPECT_EQ(digital.min_raw, DIGITAL_MIN_UNCLIPPED_RAW);
    EXPECT_EQ(digital.max_raw, DIGITAL_MAX_UNCLIPPED_RAW);
}

TEST_F(SteeringSystemTest, clipped_reading_does_not_wipe_history)
{
    _steering.updateObservedExtremes(1500, 3000);
    _steering.updateObservedExtremes(2500, 7000);

    _steering.updateObservedExtremes(CLIPPED_RAW, CLIPPED_RAW);

    EXPECT_EQ(currentParams().analog.observed_extremes.min_raw, 1500U);
    EXPECT_EQ(currentParams().analog.observed_extremes.max_raw, 2500U);
    EXPECT_EQ(currentParams().digital.observed_extremes.min_raw, 3000U);
    EXPECT_EQ(currentParams().digital.observed_extremes.max_raw, 7000U);
}

/* ============================================================================================
 * Recalibration
 * ============================================================================================ */

TEST_F(SteeringSystemTest, recalibrate_without_samples_keeps_previous_calibration)
{
    _steering.recalibrateSteering();

    const SteeringSensorParams_s &analog = currentParams().analog;
    EXPECT_EQ(analog.calibration_snapshot.min_raw, ANALOG_MIN_RAW);
    EXPECT_EQ(analog.calibration_snapshot.max_raw, ANALOG_MAX_RAW);
    EXPECT_EQ(analog.calibration_snapshot.center_raw, ANALOG_CENTER_RAW);
    EXPECT_EQ(analog.oor_bounds.min_valid_raw, ANALOG_MIN_VALID_RAW);
    EXPECT_EQ(analog.oor_bounds.max_valid_raw, ANALOG_MAX_VALID_RAW);

    const SteeringSensorParams_s &digital = currentParams().digital;
    EXPECT_EQ(digital.calibration_snapshot.min_raw, DIGITAL_MIN_RAW);
    EXPECT_EQ(digital.calibration_snapshot.max_raw, DIGITAL_MAX_RAW);
    EXPECT_EQ(digital.calibration_snapshot.center_raw, DIGITAL_CENTER_RAW);
    EXPECT_EQ(digital.oor_bounds.min_valid_raw, DIGITAL_MIN_VALID_RAW);
    EXPECT_EQ(digital.oor_bounds.max_valid_raw, DIGITAL_MAX_VALID_RAW);
}

TEST_F(SteeringSystemTest, recalibrate_computes_calibration_and_bounds)
{
    _steering.updateObservedExtremes(1500, 2000);
    _steering.updateObservedExtremes(2500, 8000);
    _steering.recalibrateSteering();

    // Analog: span 1000, tolerance 1000 * 0.005 = 5 counts
    const SteeringSensorParams_s &analog = currentParams().analog;
    EXPECT_EQ(analog.calibration_snapshot.min_raw, 1500U);
    EXPECT_EQ(analog.calibration_snapshot.max_raw, 2500U);
    EXPECT_EQ(analog.calibration_snapshot.span_raw, 1000U);
    EXPECT_EQ(analog.calibration_snapshot.center_raw, 2000U);
    EXPECT_FLOAT_EQ(analog.oor_tolerance_raw, 5.0f);
    EXPECT_EQ(analog.oor_bounds.min_valid_raw, 1495);
    EXPECT_EQ(analog.oor_bounds.max_valid_raw, 2505);

    // Digital: span 6000, tolerance 6000 * 0.005 = 30 counts
    const SteeringSensorParams_s &digital = currentParams().digital;
    EXPECT_EQ(digital.calibration_snapshot.min_raw, 2000U);
    EXPECT_EQ(digital.calibration_snapshot.max_raw, 8000U);
    EXPECT_EQ(digital.calibration_snapshot.span_raw, 6000U);
    EXPECT_EQ(digital.calibration_snapshot.center_raw, 5000U);
    EXPECT_FLOAT_EQ(digital.oor_tolerance_raw, 30.0f);
    EXPECT_EQ(digital.oor_bounds.min_valid_raw, 1970);
    EXPECT_EQ(digital.oor_bounds.max_valid_raw, 8030);
}

TEST_F(SteeringSystemTest, recalibrate_only_updates_sensors_with_samples)
{
    // Only the analog sensor gets valid samples
    _steering.updateObservedExtremes(1500, CLIPPED_RAW);
    _steering.updateObservedExtremes(2500, CLIPPED_RAW);
    _steering.recalibrateSteering();

    EXPECT_EQ(currentParams().analog.calibration_snapshot.center_raw, 2000U);
    EXPECT_EQ(currentParams().digital.calibration_snapshot.min_raw, DIGITAL_MIN_RAW);
    EXPECT_EQ(currentParams().digital.calibration_snapshot.max_raw, DIGITAL_MAX_RAW);
    EXPECT_EQ(currentParams().digital.calibration_snapshot.center_raw, DIGITAL_CENTER_RAW);
}

TEST_F(SteeringSystemTest, recalibrate_keeps_extremes_when_span_plausible)
{
    _steering.updateObservedExtremes(1500, 2000);
    _steering.updateObservedExtremes(2500, 8000);
    _steering.recalibrateSteering();

    EXPECT_EQ(currentParams().analog.observed_extremes.min_raw, 1500U);
    EXPECT_EQ(currentParams().analog.observed_extremes.max_raw, 2500U);
    EXPECT_EQ(currentParams().digital.observed_extremes.min_raw, 2000U);
    EXPECT_EQ(currentParams().digital.observed_extremes.max_raw, 8000U);
}

TEST_F(SteeringSystemTest, recalibrate_clears_extremes_when_span_implausible)
{
    // Analog span 3000 > 2500, digital span 9100 > 9000
    _steering.updateObservedExtremes(500, 100);
    _steering.updateObservedExtremes(3500, 9200);
    _steering.recalibrateSteering();

    // Extremes are reset so the next sweep starts fresh
    EXPECT_EQ(currentParams().analog.observed_extremes.min_raw, UINT32_MAX);
    EXPECT_EQ(currentParams().analog.observed_extremes.max_raw, 0U);
    EXPECT_EQ(currentParams().digital.observed_extremes.min_raw, UINT32_MAX);
    EXPECT_EQ(currentParams().digital.observed_extremes.max_raw, 0U);

    // Current behavior: the implausible calibration from this pass is still applied
    EXPECT_EQ(currentParams().analog.calibration_snapshot.min_raw, 500U);
    EXPECT_EQ(currentParams().analog.calibration_snapshot.max_raw, 3500U);
    EXPECT_EQ(currentParams().digital.calibration_snapshot.min_raw, 100U);
    EXPECT_EQ(currentParams().digital.calibration_snapshot.max_raw, 9200U);
}

TEST_F(SteeringSystemTest, conversion_and_oor_use_new_calibration)
{
    _steering.updateObservedExtremes(1500, CLIPPED_RAW);
    _steering.updateObservedExtremes(2500, CLIPPED_RAW);
    _steering.recalibrateSteering();  // new analog center 2000, bounds [1495, 2505]

    evaluateNextSample(2000, DIGITAL_CENTER_RAW);
    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), 0.0f, 0.001f);

    evaluateNextSample(2100, DIGITAL_CENTER_RAW);
    EXPECT_NEAR(_steering.getUnfilteredAnalogSteeringDeg(), 100.0f * ANALOG_DEG_PER_COUNT, 0.001f);

    // 2505 was in range under the old bounds and still is; 2506 is now out of range
    evaluateNextSample(2505, DIGITAL_CENTER_RAW);
    EXPECT_FALSE(data().analog_oor_implausibility);
    evaluateNextSample(2506, DIGITAL_CENTER_RAW);
    EXPECT_TRUE(data().analog_oor_implausibility);
}