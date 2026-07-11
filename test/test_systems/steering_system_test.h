#define STEERING_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "SteeringSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

#include <iostream>

SteeringParams_s gen_default_params(){
    SteeringParams_s params{};
    //hard code the parmas for sensors
    params.min_steering_signal_analog = 1024;
    params.max_steering_signal_analog = 3071;//actual hard coded
    
    params.min_steering_signal_digital = 25;
    params.max_steering_signal_digital = 8000; //testing values

    params.span_signal_analog = 3071;
    params.span_signal_digital = 8000;


    params.deg_per_count_analog = 0.087890625f;
    params.deg_per_count_digital = 0.02197265625f;

    // params.analog_tol = 0.005f;
    params.analog_tol_deg = 0.11377778f;
    params.digital_tol_deg = 0.2f;

    params.max_dtheta_threshold = 50.0f;//change
    params.error_between_sensors_tolerance = 0.31377778f;

    params.digital_midpoint = (params.min_steering_signal_digital + params.max_steering_signal_digital) / 2;
    params.analog_midpoint = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
   
    
    const int32_t analog_margin_counts = static_cast<int32_t>(params.analog_tol_deg / params.deg_per_count_analog);
    const int32_t digital_margin_counts = static_cast<int32_t>(params.digital_tol_deg / params.deg_per_count_digital);
    
    params.analog_min_with_margins = static_cast<int32_t>(params.min_steering_signal_analog) - analog_margin_counts;
    params.analog_max_with_margins = static_cast<int32_t>(params.max_steering_signal_analog) + analog_margin_counts;
    params.digital_min_with_margins = static_cast<int32_t>(params.min_steering_signal_digital) - digital_margin_counts;
    params.digital_max_with_margins = static_cast<int32_t>(params.max_steering_signal_digital) + digital_margin_counts;
    return params;

}


void debug_print_steering(const SteeringSystemData_s& data){
    std::cout<<"analog_steering_angle: "<<data.analog_steering_angle<<" deg\n";
    std::cout<<"digital_steering_angle: "<<data.digital_steering_angle<<" deg\n";
    std::cout<<"output_steering_angle: "<<data.output_steering_angle<<" deg\n";
    std::cout<<"analog_oor_implausibility: "<<data.analog_oor_implausibility<<"\n";
    std::cout<<"digital_oor_implausibility: "<<data.digital_oor_implausibility<<"\n";
    std::cout<<"sensor_disagreement_implausibility: "<<data.sensor_disagreement_implausibility<<"\n";
    std::cout<<"dtheta_exceeded_analog: "<<data.dtheta_exceeded_analog<<"\n";
    std::cout<<"dtheta_exceeded_digital: "<<data.dtheta_exceeded_digital<<"\n";
    

}

static SteeringEncoderReading_s hardcode_digital_data(int rawValue, SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL) {
    SteeringEncoderReading_s data{};
    data.rawValue = rawValue;
    data.status = status;
    data.angle = 0.0f; // Reset angle as it's calculated by the system, not the sensor
    data.errors = {};  // Ensure no error flags are set
    return data;
}


TEST(SteeringSystemTesting, test_adc_to_degree_conversion)
{
    auto params = gen_default_params();
    SteeringSystem steering = SteeringSystem(params);

    uint32_t analog_mid = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
    uint32_t digital_mid = (params.min_steering_signal_digital + params.max_steering_signal_digital) / 2;
    
    //midpoints
    uint32_t analog_raw = analog_mid;
    auto digital_data = hardcode_digital_data(digital_mid);

    steering.evaluate_steering(analog_raw, digital_data, 1000);
    auto data = steering.get_steering_system_data();
    data.analog_steering_angle = steering.get_unfiltered_analog_steering_deg();

    // debug_print_steering(data);
    EXPECT_NEAR(data.analog_steering_angle, 0.0f, 0.001f); 
    EXPECT_NEAR(data.digital_steering_angle, 0.0f, 0.001f);

    //min values
    analog_raw = params.min_steering_signal_analog;
    digital_data = hardcode_digital_data(params.min_steering_signal_digital);

    steering.evaluate_steering(analog_raw, digital_data, 1010);
    data = steering.get_steering_system_data();
    data.analog_steering_angle = steering.get_unfiltered_analog_steering_deg();

    float expected_analog_min = (static_cast<int32_t>(params.min_steering_signal_analog) - static_cast<int32_t>(analog_mid)) * params.deg_per_count_analog;
    float expected_digital_min = -1 * (static_cast<int32_t>(params.min_steering_signal_digital) - static_cast<int32_t>(digital_mid)) * params.deg_per_count_digital;
    
    // debug_print_steering(data);
    // std::printf("Exp A Min: %.1f\n Exp D Min: %.1f\nMin A: %d\n Min D: %d\n\n", expected_analog_min, expected_digital_min, params.min_steering_signal_analog, params.min_steering_signal_digital);
    EXPECT_NEAR(data.analog_steering_angle, expected_analog_min, 0.001f); 
    EXPECT_NEAR(data.digital_steering_angle, expected_digital_min, 0.001f); 

    //max values
    analog_raw = params.max_steering_signal_analog;
    std::printf("Analog Raw: %d\n", analog_raw);
    digital_data = hardcode_digital_data(params.max_steering_signal_digital);

    steering.evaluate_steering(analog_raw, digital_data, 1020);
    data = steering.get_steering_system_data();
    data.analog_steering_angle = steering.get_unfiltered_analog_steering_deg();

    float expected_analog_max = (static_cast<int32_t>(params.max_steering_signal_analog) - static_cast<int32_t>(analog_mid)) * params.deg_per_count_analog;
    float expected_digital_max = -1 * (static_cast<int32_t>(params.max_steering_signal_digital) - static_cast<int32_t>(digital_mid)) * params.deg_per_count_digital;

    // debug_print_steering(data);
    // std::printf("Exp A Max: %.1f\nExp D Max: %.1f\nMax A: %d\nMax D: %d\n\n", expected_analog_max, expected_digital_max, params.max_steering_signal_analog, params.max_steering_signal_digital);
    EXPECT_NEAR(data.analog_steering_angle, expected_analog_max, 0.001f); 
    EXPECT_NEAR(data.digital_steering_angle, expected_digital_max, 0.001f);
}

//FIXED
TEST(SteeringSystemTesting, test_out_of_bounds_raw_signals){
    auto params = gen_default_params();
    uint32_t analog_mid = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
    uint32_t digital_mid = (params.min_steering_signal_digital + params.max_steering_signal_digital) / 2;
    
    
    SteeringSystem steering(params);

    //Check for known valid values first
    uint32_t analog_raw = analog_mid;
    auto digital_data = hardcode_digital_data(digital_mid);
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    auto data = steering.get_steering_system_data();
    EXPECT_FALSE(data.analog_oor_implausibility);
    EXPECT_FALSE(data.digital_oor_implausibility);

    //OOR High
    analog_raw = 5000;
    digital_data = hardcode_digital_data(9000);
    steering.evaluate_steering(analog_raw, digital_data, 1010);
    data = steering.get_steering_system_data();
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility);

    //OOR Low
    
    analog_raw = static_cast<uint32_t>(params.min_steering_signal_analog) - 50;
    digital_data = hardcode_digital_data(static_cast<int>(params.min_steering_signal_digital) - 10);
    steering.evaluate_steering(analog_raw, digital_data, 1020);
    steering.evaluate_steering(analog_raw, digital_data, 1030);
    //data = steering.evaluate_steering(low_val, 1030);
    data = steering.get_steering_system_data();
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility); 

}

TEST(SteeringSystemTesting, test_detect_jumps_dtheta){
    auto params = gen_default_params();
    SteeringSystem steering(params);

    uint32_t analog_raw = 2048;
    auto digital_data = hardcode_digital_data(4000);

    //First run (just to verify that we don't get a dtheta exceeded on the first run since we have no previous data to compare to)
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    auto data = steering.get_steering_system_data();
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);
    
    analog_raw = 4096;
    digital_data = hardcode_digital_data(8000);
    steering.evaluate_steering(analog_raw, digital_data, 1005);
    data = steering.get_steering_system_data();
    EXPECT_TRUE(data.dtheta_exceeded_analog);
    EXPECT_TRUE(data.dtheta_exceeded_digital);


    // Reset the last value of evaluate steering to baseline
    analog_raw = 2048;
    digital_data = hardcode_digital_data(4000);
    steering.evaluate_steering(analog_raw, digital_data, 1010);
    data = steering.get_steering_system_data();

    std::cout << "\n angle1: " << data.digital_steering_angle;
    //Small motion valid
    analog_raw = 2060;
    digital_data = hardcode_digital_data(4050);
    steering.evaluate_steering(analog_raw, digital_data, 1120); //advance time by 110 ms
    
    data = steering.get_steering_system_data();

    std::cout << "\n angle2: " << data.digital_steering_angle;
    std::cout << "\n Velocity:" << data.digital_steering_velocity_deg_s << '\n';
    
    EXPECT_FALSE(data.dtheta_exceeded_analog);
    EXPECT_FALSE(data.dtheta_exceeded_digital);

    //Big motion NOT valid
    analog_raw = 4096;
    digital_data = hardcode_digital_data(8000);
    steering.evaluate_steering(analog_raw, digital_data, 1130); //advance time by another 10 ms
    
    data = steering.get_steering_system_data();
    EXPECT_TRUE(data.dtheta_exceeded_analog);
    EXPECT_TRUE(data.dtheta_exceeded_digital);
}


TEST(SteeringSystemTesting, test_sensor_disagreement)
{
    auto params = gen_default_params();
    SteeringSystem steering(params);

    uint32_t analog_raw = 2000;
    auto digital_data = hardcode_digital_data(4000);

    steering.evaluate_steering(analog_raw, digital_data, 1000);

    //create disagreement between sensors to test
    digital_data = hardcode_digital_data(7000); //large offset from analog

    steering.evaluate_steering(analog_raw, digital_data, 1100);
    auto data = steering.get_steering_system_data();
    EXPECT_TRUE(data.sensor_disagreement_implausibility);
} 

TEST(SteeringSystemTesting,test_sensor_output_logic){
    auto params = gen_default_params();
    

    uint32_t analog_mid = (params.min_steering_signal_analog + params.max_steering_signal_analog) / 2;
    uint32_t digital_mid = (params.min_steering_signal_digital + params.max_steering_signal_digital) / 2;
        
{
    //When both valid and agreeing, we default to digital
    SteeringSystem steering(params);
    uint32_t analog_raw = analog_mid;
    auto digital_data = hardcode_digital_data(digital_mid);
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1100);

    auto data = steering.get_steering_system_data();
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f); //probably same problem, recheck after fix
    EXPECT_FALSE(data.both_sensors_fail); //fail
    EXPECT_FALSE(data.sensor_disagreement_implausibility);
}
    // Prevent dtheta exceeded for the next test
    
{
    //When both valid but disagreeing, we default to digital
    SteeringSystem steering(params);
    uint32_t analog_raw = analog_mid;
    auto digital_data = hardcode_digital_data(digital_mid + 3000); //large offset from analog
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1100);
    auto data = steering.get_steering_system_data();

    EXPECT_TRUE(data.sensor_disagreement_implausibility);
    EXPECT_FALSE(data.analog_oor_implausibility); //actual true, expected false
    EXPECT_FALSE(data.digital_oor_implausibility); //actual true, expected false
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f); 
}
{
    //When analog is good but digital is bad, we put analog
    SteeringSystem steering(params);
    uint32_t analog_raw = analog_mid;
    auto digital_data = hardcode_digital_data(params.max_steering_signal_digital + 1000); //bad digital
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1100);
    auto data = steering.get_steering_system_data();
    EXPECT_TRUE(data.digital_oor_implausibility);
    EXPECT_FALSE(data.analog_oor_implausibility); //actual true, expected false 
    EXPECT_NEAR(data.output_steering_angle, data.analog_steering_angle, 0.001f); //propgated
}
{
    //When digital is good but analog is bad, we put digital
    SteeringSystem steering(params);
    uint32_t analog_raw = params.max_steering_signal_analog + 1000;
    auto digital_data = hardcode_digital_data(digital_mid);
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1005);
    auto data = steering.get_steering_system_data();
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_FALSE(data.digital_oor_implausibility); //actual true, expected false
    EXPECT_NEAR(data.output_steering_angle, data.digital_steering_angle, 0.001f); //propagated from 253
}
{
    //When both bad, we flagging that error
    SteeringSystem steering(params);
    uint32_t analog_raw = params.max_steering_signal_analog + 1000;
    auto digital_data = hardcode_digital_data(params.max_steering_signal_digital + 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1000);
    steering.evaluate_steering(analog_raw, digital_data, 1005);
    auto data = steering.get_steering_system_data();
    EXPECT_TRUE(data.analog_oor_implausibility);
    EXPECT_TRUE(data.digital_oor_implausibility);
    EXPECT_TRUE(data.both_sensors_fail);
}
    

}




//REWRITE THIS WHOLE THING

// TEST(SteeringSystemTesting, test_recalibrate_steering_digital)
// {
//     auto params = gen_default_params();
//     SteeringSystem steering(params);

//     // Build calibration samples from the original digital range
//     const uint32_t original_digital_min = params.min_steering_signal_digital;
//     const uint32_t original_digital_max = params.max_steering_signal_digital;
//     const uint32_t original_digital_mid = params.digital_midpoint;

//     // Pick values inside the original range so calibration observes a new min/max
//     const uint32_t observed_low  = original_digital_min + (original_digital_mid - original_digital_min) / 2;
//     const uint32_t observed_high = original_digital_mid + (original_digital_max - original_digital_mid) / 2;

//     const uint32_t analog_mid_raw = static_cast<uint32_t>(params.analog_midpoint);

//     // Start calibration: first sample should seed both observed min and max
//     steering.recalibrate_steering_digital(analog_mid_raw, observed_high, true);
//     auto updated_params = steering.get_steering_params();

//     EXPECT_EQ(updated_params.min_observed_digital, observed_high);
//     EXPECT_EQ(updated_params.max_observed_digital, observed_high);

//     // Feed lower value -> updates min only
//     steering.recalibrate_steering_digital(analog_mid_raw, observed_low, true);
//     updated_params = steering.get_steering_params();

//     EXPECT_EQ(updated_params.min_observed_digital, observed_low);
//     EXPECT_EQ(updated_params.max_observed_digital, observed_high);

//     // Feed high value again -> max remains high
//     steering.recalibrate_steering_digital(analog_mid_raw, observed_high, true);
//     updated_params = steering.get_steering_params();

//     EXPECT_EQ(updated_params.min_observed_digital, observed_low);
//     EXPECT_EQ(updated_params.max_observed_digital, observed_high);

//     // End calibration and commit recalculated values
//     steering.recalibrate_steering_digital(analog_mid_raw, observed_high, false);
//     updated_params = steering.get_steering_params();

//     // Expected committed digital range
//     const uint32_t expected_min_digital = observed_low;
//     const uint32_t expected_max_digital = observed_high;
//     const uint32_t expected_span_digital = expected_max_digital - expected_min_digital;
//     const int32_t expected_digital_midpoint = static_cast<int32_t>((expected_max_digital + expected_min_digital) / 2);

//     // Analog values should remain based on the original analog params
//     const uint32_t expected_span_analog = params.max_steering_signal_analog - params.min_steering_signal_analog;
//     const int32_t expected_analog_midpoint = static_cast<int32_t>((params.max_steering_signal_analog + params.min_steering_signal_analog) / 2);

//     const float expected_analog_tol_deg = static_cast<float>(params.span_signal_analog) *params.analog_tol *
//         params.deg_per_count_analog;

//     const int32_t expected_analog_margin_counts =
//         static_cast<int32_t>(params.analog_tol * static_cast<float>(params.span_signal_analog));

//     const int32_t expected_digital_margin_counts = static_cast<int32_t>(params.digital_tol_deg / params.deg_per_count_digital);

//     const int32_t expected_analog_min_with_margins = static_cast<int32_t>(params.min_steering_signal_analog) - expected_analog_margin_counts;
//     const int32_t expected_analog_max_with_margins = static_cast<int32_t>(params.max_steering_signal_analog) + expected_analog_margin_counts;

//     const int32_t expected_digital_min_with_margins =  static_cast<int32_t>(expected_min_digital) - expected_digital_margin_counts;
//     const int32_t expected_digital_max_with_margins = static_cast<int32_t>(expected_max_digital) + expected_digital_margin_counts;

//     const float expected_error_between_sensors_tolerance = expected_analog_tol_deg + params.digital_tol_deg;

//     // Check committed digital calibration
//     EXPECT_EQ(updated_params.min_steering_signal_digital, expected_min_digital);
//     EXPECT_EQ(updated_params.max_steering_signal_digital, expected_max_digital);
//     EXPECT_EQ(updated_params.span_signal_digital, expected_span_digital);
//     EXPECT_EQ(updated_params.digital_midpoint, expected_digital_midpoint);

//     // Check analog-derived values that get recomputed
//     EXPECT_EQ(updated_params.analog_midpoint, expected_analog_midpoint);
//     EXPECT_FLOAT_EQ(updated_params.analog_tol_deg, expected_analog_tol_deg);

//     // Check margins
//     EXPECT_EQ(updated_params.analog_min_with_margins, expected_analog_min_with_margins);
//     EXPECT_EQ(updated_params.analog_max_with_margins, expected_analog_max_with_margins);
//     EXPECT_EQ(updated_params.digital_min_with_margins, expected_digital_min_with_margins);
//     EXPECT_EQ(updated_params.digital_max_with_margins, expected_digital_max_with_margins);

//     // Check combined disagreement tolerance
//     EXPECT_FLOAT_EQ(updated_params.error_between_sensors_tolerance,
//     expected_error_between_sensors_tolerance);
// }

