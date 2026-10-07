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
    
    uint8_t fdcanStatus = FDCAN_Init();
    FDCAN_set_interfaces(CANInterfacesInstance::instance());
    initialize_status = fdCanStatus << 9 | hsStatus << 8 | tempStatus; //saves all status data
}

HT_TASK::TaskResponse checkFaults(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info) {
    uint8_t temp_alerts = 0;
    for (auto &t : temps) { temp_alerts << 1 | t.handleAlert(); } 
    //checks if alert pins were set and handled for each

    uint8_t nrsts = BuckInstance::instance().read_NRST_pins();
    hs_alert = HS5066.handleAlert();
    faults_alerted = hs_alert << 8 | alerts; //save data
    
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse gatherReadings(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info) {
    uint8_t pdb_readings.bucks_nrsts = BuckInstance::instance().read_NRST_pins();
    //read IMON Data

    float imon_currents[];
    for (auto &SW: LDSWs) { pdb_readings.imon_currents[i] = SW.getImonCurrent(); }


    return HT_TASK::TaskResponse::YIELD;
}