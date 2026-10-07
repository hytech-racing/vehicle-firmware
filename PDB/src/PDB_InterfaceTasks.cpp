#include "PDB_InterfaceTasks.hpp"


void initializeAllInterfaces() {
    VCRInterfaceInstance::create();
    CANInterfaces_s::create(VCRInterfaceInstance::instance());
    BuckInstance::create();
    
    BuckInstance::instance().init_bucks();
    uint8_t tempStatus = 0;
    for (auto& t : temps) {
        tempStatus << 1 | t.initSensor();   // keep each result
    }
    bool hsStatus = HS5066.init();
    
    FDCAN_init();
    FDCAN_set_interfaces(CANInterfacesInstance::instance());
    return (uint16_t) hsStatus << 8 | tempStatus;
}

void checkFaults() {
    uint8_t temp_alerts = 0;
    for (auto &t : temps) { temp_alerts << 1 | t.handleAlert(); } 
    //checks if alert pins were set and handled for each

    uint8_t nrsts = BuckInstance::instance().read_NRST_pins();
    hs_alert = HS5066.handleAlert();
    return (uint16_t) hs_alert << 8 | alerts;
}

void gatherReadings() {
    uint8_t nrsts = BuckInstance::instance().read_NRST_pins();
    //read IMON Data
}