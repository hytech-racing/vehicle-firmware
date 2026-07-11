#include "DrivebrainInterface.h"
#include "VCRCANInterfaceImpl.h"


DrivebrainInterface::DrivebrainInterface(IPAddress drivebrain_ip,
                                        uint16_t vcr_data_port,
                                        qindesign::network::EthernetUDP *udp_socket
) : _drivebrain_ip(drivebrain_ip),
    _vcr_data_port(vcr_data_port),
    _udp_socket(udp_socket)
{};

StampedDrivetrainCommand_s DrivebrainInterface::get_latest_telem_drivebrain_command()
{
    return _latest_drivebrain_command_telem;
}

StampedDrivetrainCommand_s DrivebrainInterface::get_latest_auxillary_drivebrain_command()
{
    return _latest_drivebrain_command_auxillary;
}

void DrivebrainInterface::receive_drivebrain_speed_command_telem(const CAN_message_t &msg, unsigned long curr_millis)
{
    DRIVEBRAIN_SPEED_SET_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command_telem.desired_speeds.recvd = true;
    _latest_drivebrain_command_telem.desired_speeds.last_recv_millis = curr_millis;

    _latest_drivebrain_command_telem.desired_speeds.veh_vec_data = {
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fr),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rr)};
};

void DrivebrainInterface::receive_drivebrain_torque_lim_command_telem(const CAN_message_t &msg, unsigned long curr_millis)
{
    DRIVEBRAIN_TORQUE_LIM_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command_telem.torque_limits.recvd = true;
    _latest_drivebrain_command_telem.torque_limits.last_recv_millis = curr_millis;

    _latest_drivebrain_command_telem.torque_limits.veh_vec_data = {
        static_cast<float>(HYTECH_drivebrain_torque_fl_ro_fromS(drivebrain_msg.drivebrain_torque_fl_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_fr_ro_fromS(drivebrain_msg.drivebrain_torque_fr_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_rl_ro_fromS(drivebrain_msg.drivebrain_torque_rl_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_rr_ro_fromS(drivebrain_msg.drivebrain_torque_rr_ro))
    };
}

void DrivebrainInterface::receive_drivebrain_speed_command_auxillary(const CAN_message_t &msg, unsigned long curr_millis)
{
    DRIVEBRAIN_SPEED_SET_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command_auxillary.desired_speeds.recvd = true;
    _latest_drivebrain_command_auxillary.desired_speeds.last_recv_millis = curr_millis;

    _latest_drivebrain_command_auxillary.desired_speeds.veh_vec_data = {
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fr),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rr)
    };
};

void DrivebrainInterface::receive_drivebrain_torque_lim_command_auxillary(const CAN_message_t &msg, unsigned long curr_millis)
{
    DRIVEBRAIN_TORQUE_LIM_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command_auxillary.torque_limits.recvd = true;
    _latest_drivebrain_command_auxillary.torque_limits.last_recv_millis = curr_millis;

    _latest_drivebrain_command_auxillary.torque_limits.veh_vec_data = {
        static_cast<float>(HYTECH_drivebrain_torque_fl_ro_fromS(drivebrain_msg.drivebrain_torque_fl_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_fr_ro_fromS(drivebrain_msg.drivebrain_torque_fr_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_rl_ro_fromS(drivebrain_msg.drivebrain_torque_rl_ro)),
        static_cast<float>(HYTECH_drivebrain_torque_rr_ro_fromS(drivebrain_msg.drivebrain_torque_rr_ro))
    };
}

void DrivebrainInterface::handle_enqueue_suspension_CAN_data(const ADCInterface &adc_instance)
{
    REAR_SUSPENSION_t rear_sus_msg;

    rear_sus_msg.rl_load_cell = adc_instance.get_filtered_RL_load_cell(); // TODO: probably want to make the CAN msg preserve some decimal points like the sus pots
    rear_sus_msg.rr_load_cell = adc_instance.get_filtered_RR_load_cell();
    rear_sus_msg.rl_shock_pot_ro = HYTECH_rl_shock_pot_ro_toS(adc_instance.get_filtered_RL_sus_pot());
    rear_sus_msg.rr_shock_pot_ro = HYTECH_rr_shock_pot_ro_toS(adc_instance.get_filtered_RR_sus_pot());

    CAN_util::enqueue_msg(&rear_sus_msg,
                        &Pack_REAR_SUSPENSION_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void DrivebrainInterface::handle_enqueue_coolant_temp_CAN_data(const ADCInterface &adc_instance)
{
    REAR_THERMISTORS_DATA_t thermistor_msg;
    thermistor_msg.thermistor_0_deg_C_ro = HYTECH_thermistor_0_deg_C_ro_toS(adc_instance.get_thermistor_n_degrees_C(0));
    thermistor_msg.thermistor_1_deg_C_ro = HYTECH_thermistor_1_deg_C_ro_toS(adc_instance.get_thermistor_n_degrees_C(1));
    thermistor_msg.thermistor_2_deg_C_ro = HYTECH_thermistor_2_deg_C_ro_toS(adc_instance.get_thermistor_n_degrees_C(2));
    CAN_util::enqueue_msg(&thermistor_msg,
                        &Pack_REAR_THERMISTORS_DATA_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void DrivebrainInterface::handle_enqueue_flowmeter_CAN_data(FlowmeterInterface &flowmeter_instance, unsigned long curr_millis)
{
    FLOWMETER_DATA_t flowmeter_msg;
    flowmeter_msg.flow_rate = static_cast<int>(flowmeter_instance.get_flow_gpm(curr_millis));
    CAN_util::enqueue_msg(&flowmeter_msg,
                        &Pack_FLOWMETER_DATA_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void DrivebrainInterface::handle_send_ethernet_data(const hytech_msgs_VCRData_s &data)
{
    handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s_size>(_drivebrain_ip,
                                                            _vcr_data_port,
                                                            _udp_socket, data,
                                                            hytech_msgs_VCRData_s_fields
    );
}
