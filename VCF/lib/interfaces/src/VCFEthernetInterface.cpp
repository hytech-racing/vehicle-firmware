#include "VCFEthernetInterface.h"
#include "hytech_msgs_version.h"


void VCFEthernetInterface::init_ethernet_device()
{
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip,
                EthernetIPDefsInstance::instance().car_subnet,
                EthernetIPDefsInstance::instance().default_gateway
    );
    _vcf_send_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
    _vcr_recv_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
}


hytech_msgs_VCFData_s VCFEthernetInterface::make_vcf_data_msg(ADCInterface &adc_int,
                                                            DashboardInterface &dash_int,
                                                            PedalsSystem &pedals_sys,
                                                            SteeringSystem &steering_sys,
                                                            BrakeRotorTempInterface &brake_rotor_temp_int
)
{
	auto fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    hytech_msgs_VCFData_s out = {};

    // has_value
    out.has_dash_input_state = true;
    out.has_front_loadcell_data = true;
    out.has_front_suspot_data = true;
    out.has_pedals_system_data = true;
    out.has_steering_data = true;
    out.has_steering_system_data = true;
    out.has_vcf_ethernet_link_data = true;
    out.has_vcf_shutdown_data = true;
    out.has_brake_pressure_data = true;
    out.has_brake_rotor_temp_data = true;

    // Load cells
    out.front_loadcell_data.FL_loadcell_analog = static_cast<uint32_t>(adc_int.get_filtered_FL_load_cell());
    out.front_loadcell_data.FR_loadcell_analog = static_cast<uint32_t>(adc_int.get_filtered_FR_load_cell());

    // Sus pots
    out.front_suspot_data.FL_sus_pot_analog = static_cast<uint32_t>(adc_int.get_filtered_FL_sus_pot());
    out.front_suspot_data.FR_sus_pot_analog = static_cast<uint32_t>(adc_int.get_filtered_FR_sus_pot());

    // Steering
    out.steering_data.analog_steering_degrees = adc_int.get_steering_degrees_cw().conversion;
    out.steering_data.digital_steering_analog = adc_int.get_steering_degrees_ccw().conversion;

    //SteeringSystem
    out.steering_system_data.analog_raw = steering_sys.get_steering_system_data().analog_raw;
    out.steering_system_data.digital_raw = steering_sys.get_steering_system_data().digital_raw;
    out.steering_system_data.analog_steering_angle = steering_sys.get_steering_system_data().analog_steering_angle;
    out.steering_system_data.digital_steering_angle = steering_sys.get_steering_system_data().digital_steering_angle;
    out.steering_system_data.output_steering_angle = steering_sys.get_steering_system_data().output_steering_angle;
    out.steering_system_data.analog_steering_velocity_deg_s = steering_sys.get_steering_system_data().analog_steering_velocity_deg_s;
    out.steering_system_data.digital_steering_velocity_deg_s = steering_sys.get_steering_system_data().digital_steering_velocity_deg_s;
    out.steering_system_data.digital_oor_implausibility = steering_sys.get_steering_system_data().digital_oor_implausibility;
    out.steering_system_data.analog_oor_implausibility = steering_sys.get_steering_system_data().analog_oor_implausibility;
    out.steering_system_data.sensor_disagreement_implausibility = steering_sys.get_steering_system_data().sensor_disagreement_implausibility;
    out.steering_system_data.dtheta_exceeded_analog = steering_sys.get_steering_system_data().dtheta_exceeded_analog;
    out.steering_system_data.dtheta_exceeded_digital = steering_sys.get_steering_system_data().dtheta_exceeded_digital;
    out.steering_system_data.both_sensors_fail = steering_sys.get_steering_system_data().both_sensors_fail;
    out.steering_system_data.interface_sensor_error = steering_sys.get_steering_system_data().interface_sensor_error;

    //TODO: MODIFY ETH STRUCT
    // Dash
    out.dash_input_state.dim_btn_is_pressed = dash_int.get_dashboard_outputs().brightness_ctrl_btn_is_pressed;
    out.dash_input_state.preset_btn_is_pressed = dash_int.get_dashboard_outputs().preset_btn_is_pressed;
    out.dash_input_state.mc_reset_btn_is_pressed = dash_int.get_dashboard_outputs().mc_reset_btn_is_pressed;
    out.dash_input_state.mode_btn_is_pressed = 0;
    out.dash_input_state.start_btn_is_pressed = dash_int.get_dashboard_outputs().start_btn_is_pressed;
    out.dash_input_state.data_btn_is_pressed = dash_int.get_dashboard_outputs().data_btn_is_pressed;
    out.dash_input_state.left_paddle_is_pressed = 0;
    out.dash_input_state.right_paddle_is_pressed = dash_int.get_dashboard_outputs().BUTTON_2;
    out.dash_input_state.dial_state = (hytech_msgs_ControllerMode_e) (dash_int.get_dashboard_outputs().dial_state);

    // Ethernet link data
    /*** Ethernet link data values initialized to 1 in VCFDataInstance_s ***/
    out.vcf_ethernet_link_data.vcr_link = 1;
    out.vcf_ethernet_link_data.teensy_link = 1;
    out.vcf_ethernet_link_data.dash_link = 1;

    // Pedals system
    out.pedals_system_data.accel_is_implausible = pedals_sys.get_pedals_system_data().accel_is_implausible;
    out.pedals_system_data.brake_is_implausible = pedals_sys.get_pedals_system_data().brake_is_implausible;
    out.pedals_system_data.brake_is_pressed = pedals_sys.get_pedals_system_data().brake_is_pressed;
    out.pedals_system_data.accel_is_pressed = pedals_sys.get_pedals_system_data().accel_is_pressed;
    out.pedals_system_data.mech_brake_is_active = pedals_sys.get_pedals_system_data().mech_brake_is_active;
    out.pedals_system_data.brake_and_accel_pressed_implausibility_high = pedals_sys.get_pedals_system_data().brake_and_accel_pressed_implausibility_high;
    out.pedals_system_data.implausibility_has_exceeded_max_duration = pedals_sys.get_pedals_system_data().implausibility_has_exceeded_max_duration;
    out.pedals_system_data.accel_percent = pedals_sys.get_pedals_system_data().accel_percent;
    out.pedals_system_data.brake_percent = pedals_sys.get_pedals_system_data().brake_percent;
    out.pedals_system_data.regen_percent = pedals_sys.get_pedals_system_data().regen_percent;

    // Brake pressure
    out.brake_pressure_data.front_brake_pressure = adc_int.get_brake_pressure_front().conversion;
    out.brake_pressure_data.rear_brake_pressure = adc_int.get_brake_pressure_rear().conversion;

    // Brake rotor temps
    out.brake_rotor_temp_data.fl_max_brake_rotor_temp = brake_rotor_temp_int.getBrakeRotorTempData().fl_sensor.max_temp;
    out.brake_rotor_temp_data.fl_avg_brake_rotor_temp = brake_rotor_temp_int.getBrakeRotorTempData().fl_sensor.avg_temp;
    out.brake_rotor_temp_data.fr_max_brake_rotor_temp = brake_rotor_temp_int.getBrakeRotorTempData().fr_sensor.max_temp;
    out.brake_rotor_temp_data.fr_avg_brake_rotor_temp = brake_rotor_temp_int.getBrakeRotorTempData().fr_sensor.avg_temp;

    // Shutdown Senses
    out.vcf_shutdown_data.d_inertia_switch_out_read = adc_int.shdn_d().conversion;
    out.vcf_shutdown_data.d_inertia_switch = adc_int.shdn_d().conversion > SHDN_HIGH_THRESHOLD ? true : false;
    out.vcf_shutdown_data.h_driver_brb_out_read = adc_int.shdn_h().conversion;
    out.vcf_shutdown_data.h_driver_brb = adc_int.shdn_h().conversion > SHDN_HIGH_THRESHOLD ? true : false;

    /* Firmware Version */
    out.has_firmware_version_info = true;
    out.firmware_version_info.project_is_dirty = device_status_t::project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    std::array<char, VER_HASH_LEN> ver_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    std::copy(ver_hash.begin(), ver_hash.end(), std::begin(out.firmware_version_info.git_hash));
    out.has_msg_versions = true;
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;

    // working with bytes in nanopb
    std::string_view version_view(version);
    const size_t version_len = std::min(version_view.size(), sizeof(out.msg_versions.ht_proto_version.bytes));
    out.msg_versions.ht_proto_version.size = version_len;
    std::copy(version_view.begin(), version_view.begin() + version_len, std::begin(out.msg_versions.ht_proto_version.bytes));

    return out;
}

void VCFEthernetInterface::receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, VCFData_s &shared_state, unsigned long curr_millis)
{
    shared_state.system_data.buzzer_is_active = msg_in.buzzer_is_active;
}

void VCFEthernetInterface::handle_send_ethernet_vcf_data(const hytech_msgs_VCFData_s &data)
{
    handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size>(
        EthernetIPDefsInstance::instance().vcr_ip,
        EthernetIPDefsInstance::instance().VCFData_port,
        &_vcf_send_socket,
        data,
        hytech_msgs_VCFData_s_fields
    );
}

#define hytech_msgs_VCFData_s_fields &hytech_msgs_VCFData_s_msg