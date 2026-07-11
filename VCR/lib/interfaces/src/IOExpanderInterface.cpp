#include "IOExpanderInterface.h"

void IOExpanderInterface::read()
{
    _data = _io_expander.read();
}

bool IOExpanderInterface::getBitPortA(uint8_t bit)
{
    return (_data >> bit) & 1;
}

bool IOExpanderInterface::getBitPortB(uint8_t bit)
{
    constexpr uint8_t BITS_IN_BYTE = 8;
    return (_data >> (BITS_IN_BYTE + bit)) & 1;
}