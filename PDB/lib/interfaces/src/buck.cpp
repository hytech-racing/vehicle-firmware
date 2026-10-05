//Read pGoods/NRSTs from the buck converters, enable and disable accordingly

/* 
Lidar - 24
Orin - 18 V
DTI Inverter - 12 V
Other peripherals - 12 V -> 5 V -> 3.3 V
Most Enable Pins are not availble due to hardware control
*/

#include "buck.h"

void BuckInterface::init_bucks(void) {
    for (auto p: pins) pinMode(p, INPUT);
}

void BuckInterface::read_NRST_pins(void) {
    for (auto p: NRST_pins) { _buck_data |= digitalRead(p); }
}

void BuckInterface::enable_bucks(void) {
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