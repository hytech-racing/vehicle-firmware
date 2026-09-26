#include "VCREthernetInterface.hpp"
#include "hytech_msgs_version.h"


void VCREthernetInterface::initEthernetDevice()
{
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().vcr_ip,
                EthernetIPDefsInstance::instance().car_subnet,
                EthernetIPDefsInstance::instance().default_gateway
    );
    _vcr_data_send_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    _vcf_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
}

hytech_msgs_VCRData_s VCREthernetInterface::makeVCRDataPBMsg(const ADCInterface &adc_interface,
                                                            DrivetrainDynamicReport_s &drivetrain_data,
                                                            const VCFInterface &vcf_interface,
                                                            const VehicleStateMachine &vehicle_state_machine,
                                                            const DrivetrainSystem &drivetrain_system,
                                                            const InverterInterface &fl_inverter,
                                                            const InverterInterface &fr_inverter,
                                                            const InverterInterface &rl_inverter,
                                                            const InverterInterface &rr_inverter,
                                                            const VCRControls &vcr_controls
)
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
    out.rear_loadcell_data.RL_loadcell_analog = static_cast<uint32_t>(adc_interface.get_filtered_RL_load_cell());
    out.rear_loadcell_data.RR_loadcell_analog = static_cast<uint32_t>(adc_interface.get_filtered_RR_load_cell());

    // RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = static_cast<uint32_t>(adc_interface.get_RL_sus_pot().conversion);
    out.rear_suspot_data.RR_sus_pot_analog = static_cast<uint32_t>(adc_interface.get_RR_sus_pot().conversion);

    // ShutdownSensingData_s
    out.vcr_shutdown_data.i_shutdown_in = false;    // shared_state.interface_data.shutdown_sensing_data.i_shutdown_in;
    out.vcr_shutdown_data.j_bspd_relay = false;     // shared_state.interface_data.shutdown_sensing_data.j_bspd_relay;
    out.vcr_shutdown_data.k_watchdog_relay = false; // shared_state.interface_data.shutdown_sensing_data.k_watchdog_relay;
    out.vcr_shutdown_data.l_bms_relay = false;      // shared_state.interface_data.shutdown_sensing_data.l_bms_relay;
    out.vcr_shutdown_data.m_imd_relay = false;      // shared_state.interface_data.shutdown_sensing_data.m_imd_relay;

    IOExpanderInterfaceInstance::instance().read();

    out.vcr_shutdown_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortA(0);       // GPA0 = BSPD_OK_SENSE
    out.vcr_shutdown_data.watchdog_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(3);   // GPB3 = VCR_OK_SENSE
    out.vcr_shutdown_data.bms_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(1);        // GPB1 = BMS_OK_SENSE
    out.vcr_shutdown_data.imd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(2);        // GPB2 = IMD_OK_SENSE

    // VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().getBitPortB(4);        // GPB4 = ACU_LINK_SENSE
    // out.ethernet_is_linked.debug_link = shared_state.interface_data.ethernet_is_linked.debug_link;   // TODO: fix this still
    out.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().getBitPortA(4); // GPA4 = DB_LINK_SENSE
    out.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().getBitPortB(5);     // GPB5 = TEENSY_LINK_SENSE
    out.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().getBitPortA(5);   // GPA5 = Ubiquiti_LINK_SENSE
    out.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().getBitPortB(6);        // GPB6 = VCF_LINK_SENSE

    // veh_vec<InverterData>
    copy_inverter_data(fl_inverter.getAllInverterData(), out.inverter_data.FL);
    out.inverter_data.has_FL = true;
    copy_inverter_data(fr_inverter.getTelemetryData(), out.inverter_data.FR);
    out.inverter_data.has_FR = true;
    copy_inverter_data(rl_inverter.getTelemetryData(), out.inverter_data.RL);
    out.inverter_data.has_RL = true;
    copy_inverter_data(rr_inverter.getTelemetryData(), out.inverter_data.RR);
    out.inverter_data.has_RR = true;

    // CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = adc_interface.get_glv().conversion;
    out.current_sensor_data.current_sensor_unfiltered = adc_interface.get_bspd_current().conversion;
    out.current_sensor_data.current_refererence_unfiltered = adc_interface.get_bspd_reference_current().conversion;
    out.current_sensor_data.bpsd_brake_high_sense = adc_interface.is_brake_sense_high();
    out.current_sensor_data.bspd_current_high_sense = adc_interface.is_current_sense_high();


    //DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = drivetrain_data.measuredInverterFLPackVoltage;

    _copyVehVecMembers(drivetrain_data.measuredSpeeds, out.drivetrain_data.measuredSpeeds);
    _copyVehVecMembers(drivetrain_data.measuredTorques, out.drivetrain_data.measuredTorques);
    _copyVehVecMembers(drivetrain_data.measuredTorqueCurrents, out.drivetrain_data.measuredTorqueCurrents);
    _copyVehVecMembers(drivetrain_data.measuredMagnetizingCurrents, out.drivetrain_data.measuredMagnetizingCurrents);

    // TorqueControllerMuxStatus
    out.tcmux_status.active_error = (hytech_msgs_TorqueControllerMuxError_e) vcr_controls.getTCMuxStatus().active_error;
    out.tcmux_status.active_controller_mode = (hytech_msgs_ControllerMode_e) vcr_controls.getTCMuxStatus().active_controller_mode;
    out.tcmux_status.active_torque_limit_value = vcr_controls.getTCMuxStatus().active_torque_limit_value;
    out.tcmux_status.output_is_bypassing_limits = vcr_controls.getTCMuxStatus().output_is_bypassing_limits;

    // Buzzer
    out.buzzer_is_active = adc_interface.get_glv().conversion;

    /* Firmware Version */
    out.has_firmware_version_info = true;
    out.firmware_version_info.project_is_dirty = device_status_t::project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    std::copy(fw_version_hash.begin(), fw_version_hash.end(), out.firmware_version_info.git_hash);
    out.has_msg_versions = true;
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;

    // working with bytes in nanopb
    std::string_view version_view(version);
    const size_t version_len = std::min(version_view.size(), sizeof(out.msg_versions.ht_proto_version.bytes));
    out.msg_versions.ht_proto_version.size = version_len;
    std::copy(version_view.begin(), version_view.begin() + version_len, std::begin(out.msg_versions.ht_proto_version.bytes));

    // VCR Status
    // const char* state_label = "UNKNOWN";
    out.status.vehicle_state = static_cast<hytech_msgs_VehicleState_e>(vehicle_state_machine.get_state());
    out.status.drivetrain_state = static_cast<hytech_msgs_DrivetrainState_e>(drivetrain_system.getCurrentState());

    out.status.drivebrain_controller_timing_failure = vcr_controls.getHasTimingFailure();
    out.status.drivebrain_is_in_control = vcr_controls.isDrivebrainInControll();

    out.status.pedals_heartbeat_ok = vcf_interface.getLatestData().stamped_pedals.heartbeat_ok;

    return out;
}

void VCREthernetInterface::receiveDrivebrainPBMsg(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.
}

void VCREthernetInterface::receiveVCFPBMsg(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
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

void VCREthernetInterface::_copyInverterData(const InverterData_s &original, hytech_msgs_InverterData_s &destination)
{
    destination.control_mode = static_cast<uint32_t>(original.control_mode);
    destination.target_iq_apk = original.target_iq_apk;
    destination.motor_position_deg = original.motor_position_deg;
    destination.is_motor_stationary = original.is_motor_stationary;

    destination.erpm = original.erpm;
    destination.duty_cycle_percent = original.duty_cycle_percent;
    destination.input_voltage = original.input_voltage;

    destination.active_ac_current_apk = original.active_ac_current_apk;
    destination.active_dc_current_amp = original.active_dc_current_amp;

    destination.controller_temp_c = original.controller_temp_c;
    destination.motor_temp_c = original.motor_temp_c;
    destination.fault_code = static_cast<uint32_t>(original.fault_code);

    destination.iq_apk = original.iq_apk;
    destination.id_apk = original.id_apk;

    destination.throttle_signal_percent = original.throttle_signal_percent;
    destination.brake_signal_percent = original.brake_signal_percent;
    destination.is_drive_enabled = original.is_drive_enabled;
    destination.can_map_version = original.can_map_version;
}

void VCREthernetInterface::_copyInverterLimits(const InverterLimits_s &original, hytech_msgs_InverterLimits_s &destination)
{
    destination.max_ac_current_apk = original.max_ac_current_apk;
    destination.available_max_ac_current_apk = original.available_max_ac_current_apk;
    destination.min_ac_current_apk = original.min_ac_current_apk;
    destination.available_min_ac_current_apk = original.available_min_ac_current_apk;

    destination.max_dc_current_amp = original.max_dc_current_amp;
    destination.available_max_dc_current_amp = original.available_max_dc_current_amp;
    destination.min_dc_current_amp = original.min_dc_current_amp;
    destination.available_min_dc_current_amp = original.available_min_dc_current_amp;

    destination.is_capacitor_temp_limit_active = original.is_capacitor_temp_limit_active;
    destination.is_dc_current_limit_active = original.is_dc_current_limit_active;
    destination.is_drive_enable_limit_active = original.is_drive_enable_limit_active;
    destination.is_igbt_accel_temp_limit_active = original.is_igbt_accel_temp_limit_active;
    destination.is_igbt_temp_limit_active = original.is_igbt_temp_limit_active;
    destination.is_input_voltage_limit_active = original.is_input_voltage_limit_active;
    destination.is_motor_accel_temp_limit_active = original.is_motor_accel_temp_limit_active;
    destination.is_motor_temp_limit_active = original.is_motor_temp_limit_active;
    destination.is_rpm_min_limit_active = original.is_rpm_min_limit_active;
    destination.is_rpm_max_limit_active = original.is_rpm_max_limit_active;
    destination.is_power_limit_active = original.is_power_limit_active;
}