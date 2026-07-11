#ifndef IOEXPANDERINTERFACE_H
#define IOEXPANDERINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <MCP23017.h>

/**
 * Config of IOExpander pins for port A/B. Each bit configures corresponding pin of the port.
 * directions: 1 = input, 0 = output / pin
 * pullups: 1 = pullup, 0 = no pullup / pin
 * inverted: 1 = inverted, 0 = normal / pin
 */
struct IOExpanderPortMode_s
{
    uint8_t directions;
    uint8_t pullups;
    uint8_t inverted;
};

struct IOExpanderInterfaceParams_s
{
    uint8_t i2c_address;
    IOExpanderPortMode_s port_a;
    IOExpanderPortMode_s port_b;
};

class IOExpanderInterface
{
public:

    IOExpanderInterface(TwoWire &wire, IOExpanderInterfaceParams_s params) : _io_expander(params.i2c_address, wire)
    {
        _io_expander.init();
        _io_expander.portMode(MCP23017Port::A, params.port_a.directions, params.port_a.pullups, params.port_a.inverted);
        _io_expander.portMode(MCP23017Port::B, params.port_b.directions, params.port_b.pullups, params.port_b.inverted);
    }

    void read();

    bool getBitPortA(uint8_t bit);
    bool getBitPortB(uint8_t bit);

    private:
    MCP23017 _io_expander;
    uint16_t _data;

};

using IOExpanderInterfaceInstance = etl::singleton<IOExpanderInterface>;

#endif