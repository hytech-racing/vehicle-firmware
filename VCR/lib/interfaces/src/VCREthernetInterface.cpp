#include "VCREthernetInterface.hpp"

#include "hytech_msgs_version.h"


void VCREthernetInterface::initEthernetDevice()
{
    EthernetIPDefsInstance::create();
    Ethernet.begin(
        EthernetIPDefsInstance::instance().vcr_ip,
        EthernetIPDefsInstance::instance().car_subnet,
        EthernetIPDefsInstance::instance().default_gateway);
    _vcr_data_send_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    _vcf_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
}

hytech_msgs_VCRData_s VCREthernetInterface::makeVCRDataMsg(
    const ADCInterface& adc_interface,
    DrivetrainDynamicReport_s& drivetrain_data,
    const VCFInterface& vcf_interface,
    const VehicleStateMachine& vehicle_state_machine,
    const DrivetrainSystem& drivetrain_system,
    const InverterInterface& fl_inverter,
    const InverterInterface& fr_inverter,
    const InverterInterface& rl_inverter,
    const InverterInterface& rr_inverter,
    const VCRControls& vcr_controls,
    float lv_bat_current_sensor_amp)
{
    auto fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    hytech_msgs_VCRData_s out;

    // has_data
    out.has_current_sensor_data = true;
    out.has_drivetrain_data = true;

    out.drivetrain_data.has_measuredMagnetizingCurrents = true;
    out.drivetrain_data.has_measuredSpeeds = true;
    out.drivetrain_data.has_measuredTorqueCurrents = true;
    out.drivetrain_data.has_measuredTorques = true;

    out.has_ethernet_is_linked = true;
    out.has_firmware_version_info = true;
    out.has_rear_loadcell_data = true;
    out.has_rear_suspot_data = true;
    out.has_vcr_shutdown_data = true;
    out.has_tcmux_status = true;
    out.has_msg_versions = true;
    out.has_status = true;

    // RearLoadCellData_s
    out.rear_loadcell_data.RL_loadcell_analog = static_cast<uint32_t>(adc_interface.getFilteredRLLoadCell());
    out.rear_loadcell_data.RR_loadcell_analog = static_cast<uint32_t>(adc_interface.getFilteredRRLoadCell());

    // RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = static_cast<uint32_t>(adc_interface.getRLSusPot().conversion);
    out.rear_suspot_data.RR_sus_pot_analog = static_cast<uint32_t>(adc_interface.getRRSusPot().conversion);

    // ShutdownSensingData_s
    out.vcr_shutdown_data.i_shutdown_in = false;    // shared_state.interface_data.shutdown_sensing_data.i_shutdown_in;
    out.vcr_shutdown_data.j_bspd_relay = false;     // shared_state.interface_data.shutdown_sensing_data.j_bspd_relay;
    out.vcr_shutdown_data.k_watchdog_relay = false; // shared_state.interface_data.shutdown_sensing_data.k_watchdog_relay;
    out.vcr_shutdown_data.l_bms_relay = false;      // shared_state.interface_data.shutdown_sensing_data.l_bms_relay;
    out.vcr_shutdown_data.m_imd_relay = false;      // shared_state.interface_data.shutdown_sensing_data.m_imd_relay;

    IOExpanderInterfaceInstance::instance().read();

    out.vcr_shutdown_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortA(0);     // GPA0 = BSPD_OK_SENSE
    out.vcr_shutdown_data.watchdog_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(3); // GPB3 = VCR_OK_SENSE
    out.vcr_shutdown_data.bms_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(1);      // GPB1 = BMS_OK_SENSE
    out.vcr_shutdown_data.imd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(2);      // GPB2 = IMD_OK_SENSE

    // VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().getBitPortB(4); // GPB4 = ACU_LINK_SENSE
    // out.ethernet_is_linked.debug_link = shared_state.interface_data.ethernet_is_linked.debug_link; // TODO: fix this still
    out.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().getBitPortA(4); // GPA4 = DB_LINK_SENSE
    out.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().getBitPortB(5);     // GPB5 = TEENSY_LINK_SENSE
    out.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().getBitPortA(5);   // GPA5 = Ubiquiti_LINK_SENSE
    out.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().getBitPortB(6);        // GPB6 = VCF_LINK_SENSE

    // VehVec_s<InverterData>
    copyInverterData(fl_inverter.getAllInverterData(), out.inverter_data.FL);
    out.inverter_data.has_FL = true;
    copyInverterData(fr_inverter.getAllInverterData(), out.inverter_data.FR);
    out.inverter_data.has_FR = true;
    copyInverterData(rl_inverter.getAllInverterData(), out.inverter_data.RL);
    out.inverter_data.has_RL = true;
    copyInverterData(rr_inverter.getAllInverterData(), out.inverter_data.RR);
    out.inverter_data.has_RR = true;

    // CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = adc_interface.getGlv().conversion;
    out.current_sensor_data.current_sensor_unfiltered = adc_interface.getBspdCurrent().conversion;
    out.current_sensor_data.current_refererence_unfiltered = adc_interface.getBspdReferenceCurrent().conversion;
    out.current_sensor_data.bpsd_brake_high_sense = adc_interface.isBrakeSenseHigh();
    out.current_sensor_data.bspd_current_high_sense = adc_interface.isCurrentSenseHigh();
    out.current_sensor_data.lv_bat_current_sensor_A = lv_bat_current_sensor_amp;


    // DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = drivetrain_data.measured_inverter_fl_pack_voltage;

    copyVehVecMembers(drivetrain_data.measured_speeds, out.drivetrain_data.measuredSpeeds);
    copyVehVecMembers(drivetrain_data.measured_torques, out.drivetrain_data.measuredTorques);
    copyVehVecMembers(drivetrain_data.measured_torque_currents, out.drivetrain_data.measuredTorqueCurrents);
    copyVehVecMembers(drivetrain_data.measured_magnetizing_currents, out.drivetrain_data.measuredMagnetizingCurrents);

    // TorqueControllerMuxStatus
    out.tcmux_status.active_error = (hytech_msgs_TorqueControllerMuxError_e)vcr_controls.getTcMuxStatus().active_error;
    out.tcmux_status.active_controller_mode = (hytech_msgs_ControllerMode_e)vcr_controls.getTcMuxStatus().active_controller_mode;
    out.tcmux_status.active_torque_limit_enum = (hytech_msgs_TorqueLimit_e)vcr_controls.getTcMuxStatus().active_torque_limit_enum;
    out.tcmux_status.active_torque_limit_value = vcr_controls.getTcMuxStatus().active_torque_limit_value;
    out.tcmux_status.output_is_bypassing_limits = vcr_controls.getTcMuxStatus().output_is_bypassing_limits;

    // Buzzer
    out.buzzer_is_active = adc_interface.getGlv().conversion;

    /* Firmware Version */
    out.has_firmware_version_info = true;
    out.firmware_version_info.project_is_dirty = device_status_t::project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    std::copy(fw_version_hash.begin(), fw_version_hash.end(), out.firmware_version_info.git_hash);
    out.has_msg_versions = true;
    out.msg_versions.ht_can_version = HT_CAN_VERSION;

    // working with bytes in nanopb
    std::string_view version_view(version);
    const size_t version_len = std::min(version_view.size(), sizeof(out.msg_versions.ht_proto_version.bytes));
    out.msg_versions.ht_proto_version.size = version_len;
    std::copy(version_view.begin(), version_view.begin() + version_len, std::begin(out.msg_versions.ht_proto_version.bytes));

    // VCR Status
    // const char* state_label = "UNKNOWN";
    out.status.vehicle_state = static_cast<hytech_msgs_VehicleState_e>(vehicle_state_machine.getState());
    out.status.drivetrain_state = static_cast<hytech_msgs_DrivetrainState_e>(drivetrain_system.getState());

    out.status.drivebrain_controller_timing_failure = vcr_controls.drivebrainTimingFailure();
    out.status.drivebrain_is_in_control = vcr_controls.drivebrainIsInControl();

    out.status.pedals_heartbeat_ok = vcf_interface.getLatestData().stamped_pedals.heartbeat_ok;

    return out;
}

void VCREthernetInterface::receivePbMsgDB(const hytech_msgs_MCUCommandData& msg_in, VCRData_s& shared_state, unsigned long curr_millis)
{
    // TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //       on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //       not know yet. Definitely worth asking Ben.
}

void VCREthernetInterface::receivePbMsgVCF(const hytech_msgs_VCFData_s& msg_in, VCRData_s& shared_state, unsigned long curr_millis)
{
    // //DashInputState_s
    // shared_state.interface_data.dash_input_state.data_btn_is_pressed = msg_in.dash_input_state.data_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.dial_state = (ControllerMode_e) msg_in.dash_input_state.dial_state;
    // shared_state.interface_data.dash_input_state.dim_btn_is_pressed = msg_in.dash_input_state.dim_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.left_paddle_is_pressed = msg_in.dash_input_state.left_paddle_is_pressed;
    // shared_state.interface_data.dash_input_state.right_paddle_is_pressed = msg_in.dash_input_state.right_paddle_is_pressed;
    // shared_state.interface_data.dash_input_state.mc_reset_btn_is_pressed = msg_in.dash_input_state.mc_reset_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.mode_btn_is_pressed = msg_in.dash_input_state.mode_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.preset_btn_is_pressed = msg_in.dash_input_state.preset_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.start_btn_is_pressed = msg_in.dash_input_state.start_btn_is_pressed;
}

void VCREthernetInterface::copyInverterData(const InverterFeedbackData_s& original, hytech_msgs_InverterData_s& destination)
{
    // Status
    destination.system_ready = original.status.system_ready;
    destination.error = original.status.error;
    destination.warning = original.status.warning;
    destination.quit_dc_on = original.status.quit_dc_on;
    destination.dc_on = original.status.dc_on;
    destination.quit_inverter_on = original.status.quit_inverter_on;
    destination.inverter_on = original.status.inverter_on;
    destination.derating_on = original.status.derating_on;
    destination.dc_bus_voltage = original.status.dc_bus_voltage;
    destination.diagnostic_number = original.status.diagnostic_number;

    // Temps
    destination.igbt_temp = original.temps.igbt_temp;
    destination.inverter_temp = original.temps.inverter_temp;
    destination.motor_temp = original.temps.motor_temp;

    // Power

    // Motor Mechanics
    destination.actual_power = original.motor_mechanics.actual_power;
    destination.actual_motor_torque = original.motor_mechanics.actual_torque;
    destination.speed_rpm = original.motor_mechanics.actual_speed;

    // Control Feedback
    destination.commanded_torque = 0; // TODO: figure out if this is actually used / updated
    destination.feedback_torque = 0;  // TODO: figure out if this is actually used / updated
}
