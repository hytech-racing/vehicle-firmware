#include "MCP23017Interface.h"


void MCP23017Interface::read()
{
    _curr_data = _io_expander.read();
}

bool MCP23017Interface::get_bit_port_a(uint8_t bit)
{
    return (_curr_data >> bit) & 1;
}

bool MCP23017Interface::get_bit_port_b(uint8_t bit)
{
    constexpr uint8_t BITS_IN_BYTE = 8;
    return (_curr_data >> (BITS_IN_BYTE + bit)) & 1;
}

void MCP23017Interface::updatePortAData()
{
    read();

    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().get_bit_port_a(0);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(1);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_fault = IOExpanderInterfaceInstance::instance().get_bit_port_a(2);
    vcr_data.interface_data.ethernet_is_linked.vn_link = IOExpanderInterfaceInstance::instance().get_bit_port_a(3);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().get_bit_port_a(4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().get_bit_port_a(5);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_missing = IOExpanderInterfaceInstance::instance().get_bit_port_a(6);
    // nothing = IOExpanderInterfaceInstance::instance().get_bit_port_a(7);
}

void MCP23017Interface::updatePortBData()
{
    read();

    // vcr_data.interface_data.shutdown_sensing_data.lv_present = IOExpanderInterfaceInstance::instance().get_bit_port_b(0);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderInterfaceInstance::instance().get_bit_port_b(1);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderInterfaceInstance::instance().get_bit_port_b(2);
    vcr_data.interface_data.shutdown_sensing_data.vcr_sw_is_ok = IOExpanderInterfaceInstance::instance().get_bit_port_b(3);
    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().get_bit_port_b(4);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().get_bit_port_b(5);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().get_bit_port_b(6);
    // nothing = IOExpanderInterfaceInstance::instance().get_bit_port_b(7);
}