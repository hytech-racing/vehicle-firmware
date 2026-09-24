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

void MCP23017Interface::writePort(MCP23017Port port, uint8_t value)
{
    _io_expander.writePort(port, value);
}