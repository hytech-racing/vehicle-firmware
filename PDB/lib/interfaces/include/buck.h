

//Read pgoods from the buck converters
//Enable stuff from it


//Perhaps set up different rails for isolation
//Best coding practices so it propagates correctly

// Lidar - 24
// Orin - 18 V
// DTI Inverter - 12 V
// Other peripheral - 12 V -> 5 V -> 3.3 V

#ifndef BUCK_H
#define BUCK_H

#include <Arduino.h>
#include <array>
#include <etl/singleton.h>

#define NRST_ORIN_18V PB10
#define NRST_DTI_12V PB12
#define NRST_MAIN_12V PB13
#define NRST_MAIN_5V PB14
#define NRST_MAIN_33V PB15

#define EN_LIDAR_24V PC8
//#define EN_ORIN_18V 
#define EN_DTI_12V PC11
//#define EN_MAIN_12V
//#define EN_MAIN_5V
//#define EN_MAIN_33V

enum class RailState { Off, Good, Fault };

struct BuckData_s {
    RailState LIDAR_24V_RAIL_STATE;
    RailState ORIN_18V_RAIL_STATE;
    RailState DTI_12V_RAIL_STATE;
    RailState MAIN_12V_RAIL_STATE;
    RailState MAIN_5V_RAIL_STATE;
    RailState MAIN_33V_RAIL_STATE;
};

class BuckInterface {
public:
    BuckInterface() {
        _buck_data = {}; // all rails default-construct to RailState::Off
    }

    void init_bucks(void);

    std::array<int, 5> read_NRST_pins(void);
    
    void enable_bucks(void);
    void enable_lidar_buck(void);
    void enable_DTI_buck(void);

    void disable_bucks(void);
    void disable_lidar_buck(void);
    void disable_DTI_buck(void);

    BuckData_s get_curr_data() { 
        return _buck_data;
    }

private:
    BuckData_s _buck_data;
};

using BuckInterfaceInstance = etl::singleton<BuckInterface>;

#endif