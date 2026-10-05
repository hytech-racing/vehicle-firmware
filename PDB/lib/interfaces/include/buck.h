

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

#define EN_LIDAR_24V PC8
#define EN_DTI_12V PC11

class BuckInterface {
public:
    const uint32_t pins[7] = {PB10, PB12, PB13, PB14, PB15, PC8, PC11};
    const uint32_t NRST_pins[5] = {PB10, PB12, PB13, PB14, PB15};
    
    BuckInterface() {
        _buck_data = {}; // all rails default-construct to RailState::Off
    }

    void init_bucks(void);

    void read_NRST_pins(void);
    
    void enable_bucks(void);
    void enable_lidar_buck(void);
    void enable_DTI_buck(void);

    void disable_bucks(void);
    void disable_lidar_buck(void);
    void disable_DTI_buck(void);

    uint8_t get_curr_data() { 
        return _buck_data;
    }

private:
    uint8_t _buck_data;
};

using BuckInterfaceInstance = etl::singleton<BuckInterface>;

#endif