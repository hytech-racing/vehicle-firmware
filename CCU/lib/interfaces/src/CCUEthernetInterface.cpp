#include "CCUEthernetInterface.h"
#include "MainChargeSystem.h"


void CCUEthernetInterface::init_ethernet_device() {
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().ccu_ip,
                EthernetIPDefsInstance::instance().car_subnet,
                EthernetIPDefsInstance::instance().default_gateway
    );
    _ccu_data_recv_socket.begin(EthernetIPDefsInstance::instance().CCUData_port);
}

void CCUEthernetInterface::receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data) {
    std::copy(std::begin(msg_in.cell_voltages), std::end(msg_in.cell_voltages), std::begin(acu_all_data.cell_voltages));
    std::copy(std::begin(msg_in.cell_temperatures), std::end(msg_in.cell_temperatures), std::begin(acu_all_data.cell_temps));
    std::copy(std::begin(msg_in.board_temperatures), std::end(msg_in.board_temperatures), std::begin(acu_all_data.board_temps));
}