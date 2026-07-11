#ifndef CHARGECYCLE_T
#define CHARGECYCLE_T

#include <gtest/gtest.h>
#include <iostream>


// TEST(mainChargeTest, calculate_charge_current_can_high_avg) { //should not charge because average cell voltage is too high
//     ACUInterfaceInstance::create(ccu_data);
//     ChargerInterfaceInstance::create(ccu_data);
//     ACUInterfaceInstance::instance().set_latest_data({7.0, 3.1, 3.3, 500}); //data is sent in the order of average, low, high, total voltage
//     ChargerInterfaceInstance::instance().set_charger_latest_data({13});
//     EXPECT_FLOAT_EQ(mainChargeLoop.calculate_charge_current(), 0); 
// }


//  TEST(mainChargeTest, bothTooHigh){ 
//      EXPECT_EQ(mainChargeLoop.calculate_charge_current(MockCCUInterface::mock_receive_message(fake_data::volts_too_much, fake_data::all_temp_high)),0);
//  };

// TEST(mainChargeTest, TempTooHigh){  
//     EXPECT_EQ(mainChargeLoop.calculate_charge_current(MockCCUInterface::mock_receive_message(fake_data::good_volts_low, fake_data::some_temp_high)),0);
// };

// TEST(mainChargeTest, VoltsTooHigh){  
//     EXPECT_EQ(mainChargeLoop.calculate_charge_current(MockCCUInterface::mock_receive_message(fake_data::more_volts_too_much, fake_data::good_temp)),0);
// };

// TEST(mainChargeTest, shouldBeTrue){  
//     EXPECT_NE(mainChargeLoop.calculate_charge_current(MockCCUInterface::mock_receive_message(fake_data::good_volts_high, fake_data::good_temp)),0);
// };


#endif