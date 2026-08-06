#include "MCP23017Interface.h"


void MCP23017IOExpanderInterface::read()
{
    _data = _io_expander.read();
}

bool MCP23017IOExpanderInterface::get_bit_port_a(uint8_t bit)
{
    return (_data >> bit) & 1;
}

bool MCP23017IOExpanderInterface::get_bit_port_b(uint8_t bit)
{
    constexpr uint8_t BITS_IN_BYTE = 8;
    return (_data >> (BITS_IN_BYTE + bit)) & 1;
}