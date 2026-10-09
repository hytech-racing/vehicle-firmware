#include "RDC_Tasks.h"
#include "CoreRDCCANInterface.h"
#include "DriverlessStateMachine.h"
#include "ht_task.hpp"

#include <cstdio>

void initializeAllInterfaces()
{
    DrivebrainInterfaceInstance::create();
    FDCInterfaceInstance::create(sys_time::hal_millis(), RDCConstants::EBS_HEARTBEAT_TIMEOUT_MS);
    RSSInterfaceInstance::create(RDCConstants::RSS_NODE_ID);
    CANInterfacesInstance::create(DrivebrainInterfaceInstance::instance(), FDCInterfaceInstance::instance(), RSSInterfaceInstance::instance());

    CoreRDCCANInterfaceInstance::create(
        etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)>::create<RDCCANInterfaceImpl::RDCRecvSwitch>(),
        RDCConstants::DRIVERLESS_CAN_BAUDRATE);

    // ADC Setup
    ADCInterfaceParams_s adc_params = {
        .pinouts =
            {.ebs_ctrl_ext_pin = 0,
             .ebs_supply_rss_ext_pin = 1,
             .ebs_supply_out_pin = 2,
             .shdn_l_pin = 3,
             .shdn_n_pin = 4,
             .ds_shdn_m_pin = 5,
             .dsms_active_ext_pin = 6,
             .shdn_ds_ok_pin = 7,
             .shdn_m_pin = 10,

             .strain_gauge_1_pin = A4,
             .strain_gauge_2_pin = A3,
             .strain_gauge_3_pin = A2,
             .strain_gauge_4_pin = A1,
             .strain_gauge_5_pin = A0},

        .channels =
            {.ebs_ctrl_ext_channel = 0,
             .ebs_supply_rss_ext_channel = 1,
             .ebs_supply_out_channel = 2,
             .shdn_l_channel = 3,
             .shdn_n_channel = 4,
             .ds_shdn_m_channel = 5,
             .dsms_active_ext_channel = 6,
             .shdn_ds_ok_channel = 7,
             .shdn_m_channel = 8,

             .strain_gauge_1_channel = 9,
             .strain_gauge_2_channel = 10,
             .strain_gauge_3_channel = 11,
             .strain_gauge_4_channel = 12,
             .strain_gauge_5_channel = 13},

        .scales =
            {.ebs_ctrl_ext_scale = 1.0f,
             .ebs_supply_rss_ext_scale = 1.0f,
             .ebs_supply_out_scale = 1.0f,
             .shdn_l_scale = 1.0f,
             .shdn_n_scale = 1.0f,
             .ds_shdn_m_scale = 1.0f,
             .dsms_active_ext_scale = 1.0f,
             .shdn_ds_ok_scale = 1.0f,
             .shdn_m_scale = 1.0f,

             .strain_gauge_1_scale = 1.0f,
             .strain_gauge_2_scale = 1.0f,
             .strain_gauge_3_scale = 1.0f,
             .strain_gauge_4_scale = 1.0f,
             .strain_gauge_5_scale = 1.0f},

        .offsets = {
            .ebs_ctrl_ext_offset = 0.0f,
            .ebs_supply_rss_ext_offset = 0.0f,
            .ebs_supply_out_offset = 0.0f,
            .shdn_l_offset = 0.0f,
            .shdn_n_offset = 0.0f,
            .ds_shdn_m_offset = 0.0f,
            .dsms_active_ext_offset = 0.0f,
            .shdn_ds_ok_offset = 0.0f,
            .shdn_m_offset = 0.0f,

            .strain_gauge_1_offset = 0.0f,
            .strain_gauge_2_offset = 0.0f,
            .strain_gauge_3_offset = 0.0f,
            .strain_gauge_4_offset = 0.0f,
            .strain_gauge_5_offset = 0.0f}};
    ADCInterfaceInstance::create(adc_params);
}

void initializeDriverless()
{

    DriverlessSystemInstance::create(
        etl::delegate<void(bool, uint8_t)>::create<FDCInterface, &FDCInterface::setFDCControl>(FDCInterfaceInstance::instance()),
        etl::delegate<DriverlessMission_e()>::create<FDCInterface, &FDCInterface::getDriverlessMision>(FDCInterfaceInstance::instance()),
        etl::delegate<EBSData_s()>::create<FDCInterface, &FDCInterface::getEbsData>(FDCInterfaceInstance::instance()),
        etl::delegate<bool()>::create<FDCInterface, &FDCInterface::getEbsSupervisorStatus>(FDCInterfaceInstance::instance()),
        etl::delegate<unsigned long()>::create<sys_time::hal_millis>(),
        etl::delegate<BrakeFluidPressureData_s()>::create<FDCInterface, &FDCInterface::getBrakeData>(FDCInterfaceInstance::instance()));

    DriverlessStateMachineInstance::create(
        etl::delegate<bool()>::create<ADCInterface, &ADCInterface::getDsmsActiveExt>(ADCInterfaceInstance::instance()),
        etl::delegate<VehicleState_e()>::create<DrivebrainInterface, &DrivebrainInterface::getVehicleState>(DrivebrainInterfaceInstance::instance()),
        etl::delegate<bool()>::create<DriverlessSystem, &DriverlessSystem::startupCheckNoTS>(DriverlessSystemInstance::instance()),
        etl::delegate<bool()>::create<DriverlessSystem, &DriverlessSystem::startupCheckTSActive>(DriverlessSystemInstance::instance()),
        etl::delegate<bool()>::create<FDCInterface, &FDCInterface::getEbsPressureOk>(FDCInterfaceInstance::instance()),
        etl::delegate<void()>::create<DriverlessSystem, &DriverlessSystem::resetNoTSValues>(DriverlessSystemInstance::instance()),
        etl::delegate<void()>::create<DriverlessSystem, &DriverlessSystem::resetTSActiveValues>(DriverlessSystemInstance::instance()));
}

HT_TASK::TaskResponse handleSendAllCanData(const unsigned long &sysMicros, const HT_TASK::TaskInfo &task_info)
{
    CoreRDCCANInterfaceInstance::instance().sendAllCanMsgs();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueFDCControlDVState(const unsigned long &sysMicros, const HT_TASK::TaskInfo &task_info)
{
    DRIVERLESS_STATUS_AND_STARTUP_t dv_status;
    dv_status.continue_toggle_supervisor = FDCInterfaceInstance::instance().getFDCControl().toggle_watchdog;
    switch (FDCInterfaceInstance::instance().getFDCControl().activate_ebs_solenoid_id)
    {
    case 1:
    {
        dv_status.activate_ebs_solenoid_1 = true;
        dv_status.activate_ebs_solenoid_2 = false;
        break;
    }
    case 2:
    {
        dv_status.activate_ebs_solenoid_1 = false;
        dv_status.activate_ebs_solenoid_2 = true;
        break;
    }
    default:
        dv_status.activate_ebs_solenoid_1 = false;
        dv_status.activate_ebs_solenoid_2 = false;
        break;
    }
    dv_status.driverless_state = static_cast<uint8_t>(DriverlessStateMachineInstance::instance().getState());
    CoreRDCCANInterfaceInstance::instance().enqueueMsg(&dv_status, &Pack_DRIVERLESS_STATUS_AND_STARTUP_ht_can);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueRSSOperatingMode(const unsigned long &sysMicros, const HT_TASK::TaskInfo &task_info)
{
    // create blank msg
    SET_OPERATING_MODE_t set_operating_mode_msg;
    set_operating_mode_msg.requested_state = OPERATIONAL_MODE_STATE;
    set_operating_mode_msg.target_node = RSSInterfaceInstance::instance().getNodeID();

    // pack and enqueue msg
    CoreRDCCANInterfaceInstance::instance().enqueueMsg(&set_operating_mode_msg, &Pack_SET_OPERATING_MODE_ht_can);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handleAsyncMain(const unsigned long &sysMicros, const HT_TASK::TaskInfo &task_info)
{
    /* Process the CAN messages we have received */
    CoreRDCCANInterfaceInstance::instance().processCANMessages(CANInterfacesInstance::instance(), sys_time::hal_millis(), CoreRDCCANInterfaceInstance::instance().recvSwitch, CANInterfaceType_e::RAUX);
    /* Collect data from ADC */
    ADCInterfaceInstance::instance().tick();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse runTickStateMachine(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info)
{
    DriverlessStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
};

HT_TASK::TaskResponse initWatchdog(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info)
{
    WatchdogInstance::create(RDCConstants::WATCHDOG_TIMEOUT_MS);
    pinMode(PinAssignments::WATCHDOG_PIN, OUTPUT);
    pinMode(PinAssignments::DS_SENSORS_OK_PIN, OUTPUT);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse runKickWatchdog(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info)
{
    digitalWrite(PinAssignments::WATCHDOG_PIN, WatchdogInstance::instance().getRDCWatchdogState(sys_time::hal_millis()));
    digitalWrite(PinAssignments::DS_SENSORS_OK_PIN, true); // TODO: implement a function instead of true that validates the ds sensors are ok and sets the ds_sensors_ok pin to that.
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse debugPrint(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info)
{
    auto &adc = ADCInterfaceInstance::instance();
    const auto &sg1 = adc.getStrainGauge1();
    const auto &sg2 = adc.getStrainGauge2();
    const auto &sg3 = adc.getStrainGauge3();
    const auto &sg4 = adc.getStrainGauge4();
    const auto &sg5 = adc.getStrainGauge5();

    // Fixed widths. Tabs slip once a name is longer than one tab stop, so shdn_ds_ok and shdn_m had no number under them.
    char line[160];
    Serial.println();
    std::snprintf(line, sizeof(line), "%-8s %-7s %-7s %-6s %-6s %-9s %-4s %-10s %-6s", "ebs_ctrl", "ebs_rss", "ebs_out", "shdn_l", "shdn_n", "ds_shdn_m", "dsms", "shdn_ds_ok", "shdn_m");
    Serial.println(line);
    std::snprintf(line, sizeof(line), "%-8d %-7d %-7d %-6d %-6d %-9d %-4d %-10d %-6d", adc.getEBSCtrlExt(), adc.getEBSSupplyRssExt(), adc.getEBSSupplyOut(), adc.getShdnL(), adc.getShdnN(),
                  adc.getDsShdnM(), adc.getDsmsActiveExt(), adc.getShdnDsOk(), adc.getShdnM());
    Serial.println(line);

    std::snprintf(line, sizeof(line), "%-8s %-8s %-8s %-8s %-8s", "sg1", "sg2", "sg3", "sg4", "sg5");
    Serial.println(line);
    std::snprintf(line, sizeof(line), "%-8d %-8d %-8d %-8d %-8d", sg1.raw, sg2.raw, sg3.raw, sg4.raw, sg5.raw);
    Serial.println(line);
    std::snprintf(line, sizeof(line), "%-8.2f %-8.2f %-8.2f %-8.2f %-8.2f", sg1.conversion, sg2.conversion, sg3.conversion, sg4.conversion, sg5.conversion);
    Serial.println(line);

    return HT_TASK::TaskResponse::YIELD;
}
