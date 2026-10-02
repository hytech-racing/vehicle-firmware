#include "CCUEthernetInterface.h"
#include "MainChargeSystem.h"


void CCUEthernetInterface::initEthernetDevice() {
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().ccu_ip,
                EthernetIPDefsInstance::instance().car_subnet,
                EthernetIPDefsInstance::instance().default_gateway
    );
    _acu_all_data_recv_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);
    _ccu_data_send_socket.begin(EthernetIPDefsInstance::instance().CCUData_port);
}

void CCUEthernetInterface::receiveACUAllData(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data) {
    std::copy(std::begin(msg_in.cell_voltages), std::end(msg_in.cell_voltages), std::begin(acu_all_data.cell_voltages));
    std::copy(std::begin(msg_in.cell_temperatures), std::end(msg_in.cell_temperatures), std::begin(acu_all_data.cell_temps));
    std::copy(std::begin(msg_in.board_temperatures), std::end(msg_in.board_temperatures), std::begin(acu_all_data.board_temps));
}

void CCUEthernetInterface::sendCCUDataMsg(const hytech_msgs_CCUData &data) {
    handle_ethernet_socket_send_pb<hytech_msgs_CCUData_size>(EthernetIPDefsInstance::instance().drivebrain_ip,
                                                                EthernetIPDefsInstance::instance().VCFData_port,
                                                                &_ccu_data_send_socket, data, hytech_msgs_CCUData_fields);
}

hytech_msgs_CCUData CCUEthernetInterface::makeCCUDataMsg() {
    ACUInterfaceData_s latest_acu_data = ACUInterfaceInstance::instance().get_latest_data();
    hytech_msgs_CCUData out;
    out.average_cell_voltage = latest_acu_data.average_voltage;
    out.min_cell_voltage = latest_acu_data.low_voltage;
    out.max_cell_voltage = latest_acu_data.high_voltage;
    out.pack_voltage = latest_acu_data.pack_voltage;
    out.min_cell_temp = latest_acu_data.min_cell_temp;
    out.max_cell_temp = latest_acu_data.max_cell_temp;
    out.avg_cell_temp = latest_acu_data.max_cell_temp;
    out.max_board_temp = latest_acu_data.max_board_temp;
    std::transform(latest_acu_data.cell_voltages.begin(),
                latest_acu_data.cell_voltages.end(),
                out.cell_voltages,
                [](const etl::optional<float> &v) { return v.value_or(0.0f); });
    std::transform(latest_acu_data.cell_temps.begin(),
               latest_acu_data.cell_temps.end(),
               out.cell_temperatures,
               [](const etl::optional<float> &v) { return v.value_or(0.0f); });
    std::transform(latest_acu_data.board_temps.begin(),
               latest_acu_data.board_temps.end(),
               out.board_temperatures,
               [](const etl::optional<float> &v) { return v.value_or(0.0f); });

    const ChargeSystemData_s &charge_system_data = MainChargeSystemInstance::instance().get_charge_data();
    out.calculated_charge_current = charge_system_data.calculated_charge_current;
    out.current_charger_state = static_cast<hytech_msgs_ChargerState_e>(charge_system_data.current_charger_state);
    out.is_120_switched = (charge_system_data.current_charger_state == ChargerState_e::CHARGING_120 ||
                            charge_system_data.current_charger_state == ChargerState_e::CHARGE_120_UNLATCHED); // this is very chud
    
    ChargerData_s charger_data = ChargerInterfaceInstance::instance().get_latest_charger_data();
    out.charger_current_output_A = ((charger_data.output_current_high >> 8) | charger_data.output_current_low) / 10.0f;
    out.charger_dc_output_V = ((charger_data.output_dc_voltage_high >> 8) | charger_data.output_dc_voltage_low) / 10.0f;
    out.charger_ac_input_V = ((charger_data.input_ac_voltage_high >> 8) | charger_data.input_ac_voltage_low) / 10.0f;;
    
    out.em_current_A = EnergyMeterInterfaceInstance::instance().get_latest_em_data().current_amps; 
    return out;
}