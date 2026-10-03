#ifndef MCP23017_INTERFACE_H
#define MCP23017_INTERFACE_H

#include "IOExpanderInterface.h"
#include "MCP23017.h"


class MCP23017Interface : public IOExpanderInterface
{
public:
    MCP23017Interface(TwoWire &wire,
                    IOExpanderParams_s params
    ) : _io_expander(params.i2c_address, wire)
    {
        _io_expander.init();
        _io_expander.portMode(MCP23017Port::A, params.port_a.directions, params.port_a.pullups, params.port_a.inverted);
        _io_expander.portMode(MCP23017Port::B, params.port_b.directions, params.port_b.pullups, params.port_b.inverted);
    }

    void read() override;
    bool get_bit_port_a(uint8_t bit) override;
    bool get_bit_port_b(uint8_t bit) override;

    void writePort(MCP23017Port port, uint8_t value);

private:

    MCP23017 _io_expander;
    uint16_t _curr_data = 0;
    
};
#endif