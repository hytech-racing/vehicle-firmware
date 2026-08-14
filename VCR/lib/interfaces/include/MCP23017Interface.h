#ifndef MCP23017_IOEXPANDER_INTERFACE_H
#define MCP23017_IOEXPANDER_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "IOExpanderInterface.h"
#include <MCP23017.h>


/**
 * Config of MCP23017 IOExpander pins for port A/B. Each bit configures corresponding pin of the port.
 * directions: 1 = input, 0 = output / pin
 * pullups: 1 = pullup, 0 = no pullup / pin
 * inverted: 1 = inverted, 0 = normal / pin
*/

class MCP23017IOExpanderInterface : public IOExpanderInterface
{
public:

    MCP23017IOExpanderInterface(TwoWire &wire,
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

private:

    MCP23017 _io_expander;
    uint16_t _data;

};

using IOExpanderInterfaceInstance = etl::singleton<MCP23017IOExpanderInterface>;
#endif