//Read pGoods from the buck converters
//Enable stuff from it

//Perhaps set up different rails for isolation
//Best coding practices so it propagates correctly

// Lidar - 24
// Orin - 18 V
// DTI Inverter - 12 V
// Other peripheral - 12 V -> 5 V -> 3.3 V

#include "buck.h"

void BuckInterface::init_bucks(void) {
    pinMode(NRST_ORIN_18V, INPUT);
    pinMode(NRST_DTI_12V, INPUT);
    pinMode(NRST_MAIN_12V, INPUT);
    pinMode(NRST_MAIN_5V, INPUT);
    pinMode(NRST_MAIN_33V, INPUT);

    //define the pins for the enables
    pinMode(EN_LIDAR_24V, OUTPUT);
    pinMode(EN_DTI_12V, OUTPUT);

    // Leaving these pins for future enable pin compatibility
    // pinMode(EN_ORIN_18V, OUTPUT);
    // pinMode(EN_MAIN_12V, OUTPUT);
    // pinMode(EN_MAIN_5V, OUTPUT);
    // pinMode(EN_MAIN_33V, OUTPUT);
}

// Two Questions: Do I switch off the load switches, Do I work backward in the rail
// For load switches, I personally think it should be handled in main with the read function
// I doubt backward as it would require a full shutdown

// Another question: Do I constantly read, or should I set up an interrupt
// Answer (Research + Claude): Not interrupt driven, since PGOOD can be glitchy, and 
// microsecond precision not required for this system, so polling works.

std::array<int, 5> BuckInterface::read_NRST_pins(void) {
    int i = digitalRead(NRST_ORIN_18V);
    _buck_data.ORIN_18V_RAIL_STATE = (i == 1) ? RailState::Good : RailState::Fault;
    int j = digitalRead(NRST_DTI_12V);
    _buck_data.DTI_12V_RAIL_STATE = (j == 1) ? RailState::Good : RailState::Fault;
    int k = digitalRead(NRST_MAIN_12V);
    _buck_data.MAIN_12V_RAIL_STATE = (k == 1) ? RailState::Good : RailState::Fault;
    int l = digitalRead(NRST_MAIN_5V);
    _buck_data.MAIN_5V_RAIL_STATE = (l == 1) ? RailState::Good : RailState::Fault;
    int m = digitalRead(NRST_MAIN_33V);
    _buck_data.MAIN_33V_RAIL_STATE = (m == 1) ? RailState::Good : RailState::Fault;

    std::array<int,5> NRST = {i, j, k, l, m};
    return NRST;
}

void BuckInterface::enable_bucks(void) {
    read_NRST_pins(); //gets the most updated readings, just as a precaution
    digitalWrite(EN_LIDAR_24V, 1); //Default on, unless turned off
    digitalWrite(EN_DTI_12V, 1);
}

void BuckInterface::enable_lidar_buck(void) {
    digitalWrite(EN_LIDAR_24V, 1);
}

void BuckInterface::enable_DTI_buck(void) {
    digitalWrite(EN_DTI_12V, 1);
}

void BuckInterface::disable_bucks(void) {
    digitalWrite(EN_LIDAR_24V, 0);
    digitalWrite(EN_DTI_12V, 0);
}

void BuckInterface::disable_lidar_buck(void) {
    digitalWrite(EN_LIDAR_24V, 0);
}

void BuckInterface::disable_DTI_buck(void) {
    digitalWrite(EN_DTI_12V, 0);
}




/*
This code assumed we had access to all EN pins, but apparently it was hardware wired
For future firmware drivers

void enable_bucks(void) {
    read_NRST_pins(); //gets the most updated readings

    //when to enable the LIDAR buck (no PGood signal)
    digitalWrite(EN_LIDAR_24V, 1);
    digitalWrite(EN_ORIN_18V, 1);
    digitalWrite(EN_DTI_12V, 1);
    digitalWrite(EN_MAIN_12V, 1);
    
    //checks if 12V rail is fine before connecting buck converter
    if (MAIN_12V_RAIL_STATE == RailState::Good){
        digitalWrite(EN_MAIN_5V, 1);
    }
    else {
        digitalWrite(EN_MAIN_5V, 0);
    }

    //checks if 12V and 5V rail is fine before connecting buck converter
    if (MAIN_12V_RAIL_STATE == RailState::Good && MAIN_5V_RAIL_STATE == RailState::Good) {
        digitalWrite(EN_MAIN_33V, 1);
    }
    else {
        digitalWrite(EN_MAIN_33V, 0);
    }
}
*/