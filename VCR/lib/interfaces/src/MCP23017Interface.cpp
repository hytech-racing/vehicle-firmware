#include "MCP23017Interface.hpp"


void MCP23017Interface::read()
{
    _curr_data = _io_expander.read();
}

bool MCP23017Interface::getBitPortA(uint8_t bit)
{
    return (_curr_data >> bit) & 1;
}

bool MCP23017Interface::getBitPortB(uint8_t bit)
{
    constexpr uint8_t BITS_IN_BYTE = 8;
    return (_curr_data >> (BITS_IN_BYTE + bit)) & 1;
}

void MCP23017Interface::updatePortAData()
{
    read();

    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortA(0);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(1);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_fault = IOExpanderInterfaceInstance::instance().getBitPortA(2);
    vcr_data.interface_data.ethernet_is_linked.vn_link = IOExpanderInterfaceInstance::instance().getBitPortA(3);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().getBitPortA(4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().getBitPortA(5);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_missing = IOExpanderInterfaceInstance::instance().getBitPortA(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(7);
}

void MCP23017Interface::updatePortBData()
{
    read();

    // vcr_data.interface_data.shutdown_sensing_data.lv_present = IOExpanderInterfaceInstance::instance().getBitPortB(0);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(1);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(2);
    vcr_data.interface_data.shutdown_sensing_data.vcr_sw_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(3);
    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().getBitPortB(4);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().getBitPortB(5);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().getBitPortB(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortB(7);
}