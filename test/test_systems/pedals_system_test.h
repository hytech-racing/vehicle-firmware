#define PEDALS_SYSTEM_TEST
#include <gtest/gtest.h>
#include <string>
#include "PedalsSystem.h"
#include "SharedFirmwareTypes.h"
#include <array>

#include <iostream>

float get_pedal_conversion_val(float min, float max, float data)
{
    float scale = 1.0f / (max - min);
    return ((data - min) * scale);
}

PedalsParams gen_positive_and_negative_slope_params()
{
    PedalsParams params;
    params.min_pedal_1 = 90;
    params.max_pedal_1 = 4000;
    params.min_pedal_2 = 4000;
    params.max_pedal_2 = 90;
    params.activation_percentage = 0.25;
    params.min_sensor_pedal_1 = 90;
    params.min_sensor_pedal_2 = 90;
    params.max_sensor_pedal_1 = 4000;
    params.max_sensor_pedal_2 = 4000;
    params.deadzone_margin = .03;
    params.implausibility_margin = 0.1;
    params.mechanical_activation_percentage = 0.4;
    return params;
}

PedalsParams gen_positive_slope_only_params()
{
    PedalsParams params;
    params.min_pedal_1 = 90;
    params.max_pedal_1 = 4000;
    params.min_pedal_2 = 90;
    params.max_pedal_2 = 4000;
    params.min_sensor_pedal_1 = 90;
    params.max_sensor_pedal_1 = 4000;
    params.min_sensor_pedal_2 = 90;
    params.max_sensor_pedal_2 = 4000;
    params.activation_percentage = 0.1;
    params.mechanical_activation_percentage = 0.4;
    params.deadzone_margin = 0.05;
    params.implausibility_margin = 0.1;
    return params;
}

void debug_print_pedals(PedalsSystemData_s data)
{
    std::cout << "accel_is_implausible "<< data.accel_is_implausible << std::endl;
    std::cout << "brake_is_implausible "<< data.brake_is_implausible << std::endl;
    std::cout << "brake_is_pressed "<< data.brake_is_pressed << std::endl;
    std::cout << "accel_is_pressed "<< data.accel_is_pressed << std::endl;
    std::cout << "mech_brake_is_active "<< data.mech_brake_is_active << std::endl;
    std::cout << "brake_and_accel_pressed_implausibility_high "<< data.brake_and_accel_pressed_implausibility_high << std::endl;
    std::cout << "implausibility_has_exceeded_max_duration "<< data.implausibility_has_exceeded_max_duration << std::endl;
    std::cout << "accel_percent "<< data.accel_percent << std::endl;
    std::cout << "brake_percent "<< data.brake_percent << std::endl;
    std::cout << "regen_percent "<< data.regen_percent << std::endl;
}

// returns true if implausibility has exceeded max duration for double brake test
bool get_result_of_double_brake_test(PedalsSystem &pedals, const PedalSensorData_s &sensor_data)
{
    pedals.evaluate_pedals(sensor_data, 1000);
    pedals.evaluate_pedals(sensor_data, 1110);
    return pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration;
}

// resets implausibility time and returns true always
bool reset_pedals_system_implaus_time(PedalsSystem &pedals)
{
    // Populating the test data with plausible values for the pedals
    PedalSensorData_s test_pedal_data;
    test_pedal_data.accel_1 = 94;
    test_pedal_data.accel_2 = 3996;
    test_pedal_data.brake_1 = 94;
    test_pedal_data.brake_2 = 3996;

    pedals.evaluate_pedals(test_pedal_data, 1000);
    pedals.evaluate_pedals(test_pedal_data, 1110);

    // Always returns true because the plausible values were used
    return (!pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration);
}

TEST(PedalsSystemTesting, test_good_pedals)
{
    PedalsParams accel_params;
    accel_params.min_pedal_1 = 2000;
    accel_params.max_pedal_1 = 4000;
    accel_params.min_pedal_2 = 4000;
    accel_params.max_pedal_2 = 2000;
    accel_params.activation_percentage = 0.05;
    accel_params.min_sensor_pedal_1 = 1000;
    accel_params.min_sensor_pedal_2 = 1000;
    accel_params.max_sensor_pedal_1 = 5000;
    accel_params.max_sensor_pedal_2 = 5000;
    accel_params.deadzone_margin = .03;
    accel_params.implausibility_margin = 0.1;
    accel_params.mechanical_activation_percentage = 0.4;

    PedalsParams brake_params;
    brake_params.min_pedal_1 = 2000;
    brake_params.max_pedal_1 = 4000;
    brake_params.min_pedal_2 = 2000;
    brake_params.max_pedal_2 = 4000;
    brake_params.activation_percentage = 0.05;
    brake_params.min_sensor_pedal_1 = 1000;
    brake_params.min_sensor_pedal_2 = 1000;
    brake_params.max_sensor_pedal_1 = 5000;
    brake_params.max_sensor_pedal_2 = 5000;
    brake_params.deadzone_margin = .03;
    brake_params.implausibility_margin = 0.1;
    brake_params.mechanical_activation_percentage = 0.4;
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s sense_data = {accel_params.min_pedal_1, accel_params.min_pedal_2, brake_params.min_pedal_1, brake_params.min_pedal_2};
    pedals.evaluate_pedals(sense_data, 1000);
    debug_print_pedals(pedals.get_pedals_system_data());

    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);
    sense_data = {3000, 3000, brake_params.min_pedal_1, brake_params.min_pedal_2};
    pedals.evaluate_pedals(sense_data, 1010);
    debug_print_pedals(pedals.get_pedals_system_data());

    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.5, 0.001);
}


// T.4.3.4, T.4.2.7, T.4.2.10 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_accel_and_brake_limits_plausibility)
{
    auto params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(params, params);

    // Create test pedal data
    PedalSensorData_s test_pedal_good_val = {2045, 2045, 94, 3996}; // All values in a good range

    // Create a vector of test cases for acceleration sensor implausibility
    std::vector<PedalSensorData_s> accel_test_cases = {
        {0, 5000, 94, 3996},
        {5000, 0, 94, 3996}

    };

    // Create a vector of test cases for brake sensor implausibility
    std::vector<PedalSensorData_s> brake_test_cases = {
        {94, 3996, 0, 5000},
        {94, 3996, 5000, 0}
    };

    // T.4.2.4 and T.4.2.10 (accel out of ranges min/max) testing
    for (const auto& test : accel_test_cases)
    {
        // Test double brake mode
        bool t_4_2_7 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_2_7); // Expecting implausibility duration to be exceed for all because all values are out of range and beyond 100 ms duration
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

    // Ensure that all good is still good for double brake mode
    bool t_4_2_7 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_2_7);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));

    // // T.4.3.4 brake testing
    for (const auto& test : brake_test_cases)
    {
        // Test double brake mode
        bool t_4_3_4 = get_result_of_double_brake_test(pedals, test);
        EXPECT_TRUE(t_4_3_4);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }

    // // Ensure that all good is still good for double brake mode
    bool t_4_3_4 = get_result_of_double_brake_test(pedals, test_pedal_good_val);
    EXPECT_FALSE(t_4_3_4);
    EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
}

// T.4.2.4 FSAE rules 2025 v1 (accel vals not within 10 percent of each other)
TEST(PedalsSystemTesting, test_accel_and_brake_percentages_implausibility)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_half_pressed_one = {2045, 94, 94, 3996};
    PedalSensorData_s test_pedal_half_pressed_two = {94, 2045, 94, 3996};
    PedalSensorData_s test_pedal_not_pressed = {94, 3996, 94, 3996};
    PedalSensorData_s test_pedal_half_pressed = {2045, 2045, 94, 3996};

    std::vector<std::tuple<PedalSensorData_s, bool>> test_cases = {
        {test_pedal_half_pressed_one, true},
        {test_pedal_half_pressed_two, true},
        {test_pedal_not_pressed, false},
        {test_pedal_half_pressed, false},
    };

    for (const auto& test_case : test_cases)
    {
        const auto& sensor_data = std::get<0>(test_case);
        bool expected_result = std::get<1>(test_case);

        bool res = get_result_of_double_brake_test(pedals, sensor_data);
        EXPECT_EQ(res, expected_result);
        EXPECT_TRUE(reset_pedals_system_implaus_time(pedals));
    }
}

// EV.4.7.1 and EV 4.7.2 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_accel_and_brake_pressed_at_same_time_and_activation)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // testing with example half pressed values
    PedalSensorData_s test_pedal_val_half_pressed = {2045, 2045, 2045, 2045};

    pedals.evaluate_pedals(test_pedal_val_half_pressed, 1000);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_pressed);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_is_pressed);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_and_accel_pressed_implausibility_high);
    
    // remove pressing both pedals
    PedalSensorData_s test_pedal_val_unpressed = {94, 3996, 94, 3996};
    pedals.evaluate_pedals(test_pedal_val_unpressed, 1100);
    EXPECT_FALSE(pedals.get_pedals_system_data().accel_is_pressed);
    EXPECT_FALSE(pedals.get_pedals_system_data().brake_is_pressed);
    EXPECT_FALSE(pedals.get_pedals_system_data().brake_and_accel_pressed_implausibility_high);

    // Future Implementation: test with real values from the car
}

// T.4.2.5 FSAE rules 2025 v1
TEST(PedalsSystemTesting, test_implausibility_duration)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_data = {2045, 2045, 2045, 2045};
    
    // Testing accel and brake pressed together
    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_and_accel_pressed_implausibility_high);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_is_pressed);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_pressed);
    EXPECT_FALSE(pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration);

    pedals.evaluate_pedals(test_pedal_data, 1110);
    EXPECT_TRUE(pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration);
}

// EV.4.7.2 a FSAE rules 2025 v1
TEST(PedalsSystemTesting, implausibility_latching_until_accel_released_double_brake)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // create test data with both pedals pressed
    PedalSensorData_s test_pedal_data = {2045, 2045, 2045, 2045};

    // create an implausibility in the acceleration pedal
    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_data));

    // make acceleration pedal go out of range
    test_pedal_data.accel_1 = 5000;
    test_pedal_data.accel_2 = 0;
    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_data));

    // Release acceleration pedal and ensure implausibility sets to false
    test_pedal_data.accel_1 = 94;
    test_pedal_data.accel_2 = 3996;
    test_pedal_data.brake_1 = 94;
    test_pedal_data.brake_2 = 3996;
    EXPECT_FALSE(get_result_of_double_brake_test(pedals, test_pedal_data));
}

TEST(PedalsSystemTesting, implausibility_latching_and_accel_is_zero)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // create test data with both pedals pressed
    PedalSensorData_s test_pedal_data = {2045, 2045, 2045, 2045};

    // create an implausibility in the acceleration pedal
    EXPECT_TRUE(get_result_of_double_brake_test(pedals, test_pedal_data));

    pedals.evaluate_pedals(test_pedal_data, 1200);
    // PedalSensorData_s test_not_pressed_pedal_data = {accel_params., 2045, 2045, 2045};
    PedalSensorData_s test_not_pressed_pedal_data;
    test_not_pressed_pedal_data.accel_1 = accel_params.max_pedal_1-1;
    test_not_pressed_pedal_data.accel_2 = accel_params.max_pedal_2-1;

    test_not_pressed_pedal_data.brake_1 = brake_params.max_pedal_1-1;
    test_not_pressed_pedal_data.brake_2 = brake_params.max_pedal_2-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1200);

    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0);


    test_not_pressed_pedal_data.accel_1 = accel_params.max_pedal_1-50;
    test_not_pressed_pedal_data.accel_2 = accel_params.max_pedal_2-50;

    test_not_pressed_pedal_data.brake_1 = brake_params.min_pedal_1-1;
    test_not_pressed_pedal_data.brake_2 = brake_params.min_pedal_1-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1200);

    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0);

    test_not_pressed_pedal_data.accel_1 = accel_params.max_pedal_1-50;
    test_not_pressed_pedal_data.accel_2 = accel_params.max_pedal_2-50;

    test_not_pressed_pedal_data.brake_1 = brake_params.min_pedal_1-1;
    test_not_pressed_pedal_data.brake_2 = brake_params.min_pedal_1-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1200);

    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0);

    test_not_pressed_pedal_data.accel_1 = accel_params.max_pedal_1-50;
    test_not_pressed_pedal_data.accel_2 = accel_params.max_pedal_2-50;

    test_not_pressed_pedal_data.brake_1 = brake_params.min_pedal_1-1;
    test_not_pressed_pedal_data.brake_2 = brake_params.min_pedal_1-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1200);

    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0);
    
    // this should reset the error
    test_not_pressed_pedal_data.accel_1 = accel_params.min_pedal_1+1;
    test_not_pressed_pedal_data.accel_2 = accel_params.min_pedal_2-1;

    test_not_pressed_pedal_data.brake_1 = brake_params.min_pedal_1+1;
    test_not_pressed_pedal_data.brake_2 = brake_params.min_pedal_2-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1210);

    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0);

    test_not_pressed_pedal_data.accel_1 = accel_params.max_pedal_1-1;
    test_not_pressed_pedal_data.accel_2 = accel_params.max_pedal_2+1;

    test_not_pressed_pedal_data.brake_1 = brake_params.min_pedal_1+1;
    test_not_pressed_pedal_data.brake_2 = brake_params.min_pedal_2-1;
    pedals.evaluate_pedals(test_not_pressed_pedal_data, 1220);
    debug_print_pedals(pedals.get_pedals_system_data());
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 1);

}

// testing accel and brake pedal percentage accuracies with deadzone
TEST(PedalsSystemTesting, deadzone_removal_calc_double_brake_ped)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    // Test accel pedal with good values (0%, 100%)
    PedalSensorData_s test_pedal_data = {94, 3996, 94, 3996};
    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);

    test_pedal_data = {2045, 2045, 94, 3996};
    pedals.evaluate_pedals(test_pedal_data, 1100);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.5, 0.001);

    test_pedal_data = {3996, 94, 94, 3996};
    pedals.evaluate_pedals(test_pedal_data, 1200);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 1, .001);

    // Testing brake pedal with good values (0%, 50%, 100%)
    PedalSensorData_s test_brake_pedal_data = {94, 3996, 94, 3996};
    pedals.evaluate_pedals(test_brake_pedal_data, 1000);
    EXPECT_NEAR(pedals.get_pedals_system_data().brake_percent, 0.0, 0.001);

    test_brake_pedal_data = {94, 3996, 2045, 2045};
    pedals.evaluate_pedals(test_brake_pedal_data, 1100);
    EXPECT_NEAR(pedals.get_pedals_system_data().brake_percent, 0.5, .001);

    test_brake_pedal_data = {94, 3996, 3996, 94};
    pedals.evaluate_pedals(test_brake_pedal_data, 1200);
    EXPECT_NEAR(pedals.get_pedals_system_data().brake_percent, 1, .001);

}

// testing mechanical brake and brake activation
TEST(PedalsSystemTesting, brake_value_testing_double)
{
    PedalSensorData_s test_pedal_data = {94, 3996, 872, 3218};
    auto params = gen_positive_and_negative_slope_params();
    
    params.deadzone_margin = 0;
    PedalsSystem pedals(params, params);

    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_NEAR(pedals.get_pedals_system_data().brake_percent, 0.2, 0.001);
    EXPECT_FALSE(pedals.get_pedals_system_data().brake_is_pressed);
    EXPECT_FALSE(pedals.get_pedals_system_data().mech_brake_is_active);

    test_pedal_data = {94, 3996, 2045, 2045};
    pedals.evaluate_pedals(test_pedal_data, 1100);
    EXPECT_NEAR(pedals.get_pedals_system_data().brake_percent, 0.5, 0.001);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_is_pressed);
    EXPECT_TRUE(pedals.get_pedals_system_data().mech_brake_is_active);
}

// checking to see that accel pedal can never be negative
TEST(PedalsSystemTesting, check_accel_never_negative_double)
{
    PedalSensorData_s test_pedal_data = {static_cast<uint32_t>(-1000), static_cast<uint32_t>(-3000), 94, 3996};
    auto params = gen_positive_and_negative_slope_params();

    PedalsSystem pedals(params, params);
    params.deadzone_margin = 0;

    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_implausible);
    EXPECT_EQ(pedals.get_pedals_system_data().accel_percent, 0.0);
}

// testing that accel pedal marks as pressed
TEST(PedalsSystemTesting, check_accel_pressed)
{
    PedalSensorData_s test_pedal_data = {2045, 2045, 94, 3996};
    auto params = gen_positive_and_negative_slope_params();
    params.deadzone_margin = .0;

    PedalsSystem pedals(params, params);

    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_pressed);  


    

    test_pedal_data = {872, 3218, 91, 3900};
    PedalsSystem pedals2(params, params);
    pedals2.evaluate_pedals(test_pedal_data, 1000);
    debug_print_pedals(pedals2.get_pedals_system_data());
    EXPECT_FALSE(pedals2.get_pedals_system_data().accel_is_pressed);
    EXPECT_NEAR(pedals2.get_pedals_system_data().accel_percent, 0.2, 0.001);
    
    test_pedal_data = {2145,1945,94,3996};
    PedalsSystem pedals3(params,params);
    pedals3.evaluate_pedals(test_pedal_data,1000);
    EXPECT_TRUE(pedals3.get_pedals_system_data().accel_is_pressed);

    test_pedal_data = {params.min_pedal_1+2 ,params.min_pedal_2 - 4, params.min_pedal_1 - 4,params.min_pedal_2};
    PedalsSystem pedals4(params, params);
    pedals4.evaluate_pedals(test_pedal_data,1000);
    EXPECT_FALSE(pedals4.get_pedals_system_data().accel_is_pressed);
    
    debug_print_pedals(pedals4.get_pedals_system_data());

    test_pedal_data = {94,3996,94,3996};
    PedalsSystem pedals5(params,params);
    pedals5.evaluate_pedals(test_pedal_data,1000);
    EXPECT_FALSE(pedals5.get_pedals_system_data().accel_is_pressed);

    test_pedal_data = {194,3896,94,3996};
    PedalsSystem pedals6(params,params);
    pedals6.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_FALSE(pedals6.get_pedals_system_data().accel_is_pressed);

    test_pedal_data = {294, 3796, 94, 3996};
    PedalsSystem pedals8(params,params);
    pedals8.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_FALSE(pedals8.get_pedals_system_data().accel_is_pressed);

    test_pedal_data = {1094, 2996, 94, 3996};
    PedalsSystem pedals9(params,params);
    pedals9.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals9.get_pedals_system_data().accel_is_pressed);

    test_pedal_data = {1194,2896,04,3996};
    PedalsSystem pedals7(params,params);
    pedals7.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals7.get_pedals_system_data().accel_is_pressed);

}

TEST(PedalsSystemTesting, check_brake_pressed)
{
    PedalSensorData_s test_pedal_data = {94, 3996, 2045, 2045};
    auto params = gen_positive_and_negative_slope_params();
    params.deadzone_margin = .0;

    PedalsSystem pedals(params, params);

    pedals.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_TRUE(pedals.get_pedals_system_data().brake_is_pressed);    

    // Is supposed to fail, will be 0.2
    test_pedal_data = {90,3900,872,3218}; 
    PedalsSystem pedals2(params, params);
    pedals2.evaluate_pedals(test_pedal_data, 1000);
    EXPECT_FALSE(pedals2.get_pedals_system_data().brake_is_pressed);
    EXPECT_NEAR(pedals2.get_pedals_system_data().brake_percent, 0.2, 0.001);
}

// testing that accel percent and accel implaus is marked when pedals are out of range
TEST(PedalsSystemTesting, check_accel_oor)
{
    PedalSensorData_s test_pedal_oor_hi_val_accel = {5000, 0, 94, 3996};
    PedalSensorData_s test_pedal_oor_lo_val_accel = {0, 5000, 94, 3996};

    auto params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(params, params);

    pedals.evaluate_pedals(test_pedal_oor_hi_val_accel, 1000);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_implausible);
    pedals.evaluate_pedals(test_pedal_oor_lo_val_accel, 1000);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_implausible);
}

// T.4.2.4 FSAE rules 2025 v1 (accel vals not within 10 percent of each other) and no time has passed
TEST(PedalsSystemTesting, test_accel_and_brake_percentages_implausibility_immediate_zero)
{
    auto accel_params = gen_positive_and_negative_slope_params();
    auto brake_params = gen_positive_and_negative_slope_params();
    PedalsSystem pedals(accel_params, brake_params);

    PedalSensorData_s test_pedal_half_pressed_one = {2045, 94, 94, 3996};
    PedalSensorData_s test_pedal_half_pressed = {2045, 2045, 94, 3996};
    PedalSensorData_s test_pedal_not_pressed = {94, 3996, 94, 3996};

    pedals.evaluate_pedals(test_pedal_half_pressed_one, 1000);

    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_implausible);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);
    // ensure that even if we go back to being plausible we are latching the fault 
    pedals.evaluate_pedals(test_pedal_half_pressed, 1000);
    EXPECT_FALSE(pedals.get_pedals_system_data().accel_is_implausible);
    // this is false, but accel should still be zero
    EXPECT_FALSE(pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);

    // ensure that even as time passes the implaus stays
    pedals.evaluate_pedals(test_pedal_half_pressed_one, 2000);
    EXPECT_TRUE(pedals.get_pedals_system_data().accel_is_implausible);
    EXPECT_NEAR(pedals.get_pedals_system_data().accel_percent, 0.0, 0.001);
    
    pedals.evaluate_pedals(test_pedal_not_pressed, 3000);
    
    EXPECT_FALSE(pedals.get_pedals_system_data().accel_is_implausible);
    EXPECT_FALSE(pedals.get_pedals_system_data().implausibility_has_exceeded_max_duration);

    // ensure that 
}