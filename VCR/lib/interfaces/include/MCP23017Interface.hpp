#ifndef MCP23017_IOEXPANDER_INTERFACE
#define MCP23017_IOEXPANDER_INTERFACE

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "IOExpanderInterface.h"
#include <MCP23017.h>

/* Local Includes */
#include "VCR_Globals.hpp"


/**
 * Config of MCP23017 IOExpander pins for port A/B. Each bit configures corresponding pin of the port.
 * directions: 1 = input, 0 = output / pin
 * pullups: 1 = pullup, 0 = no pullup / pin
 * inverted: 1 = inverted, 0 = normal / pin
*/
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

    bool getBitPortA(uint8_t bit) override;

    bool getBitPortB(uint8_t bit) override;

    /**
     * @brief Method updates data pertaining to port A
    */
    void updatePortAData();

    /**
     * @brief Method updates data pertaining to port B
    */
    void updatePortBData();

private:

    MCP23017 _io_expander;
    uint32_t _curr_data;

};

using IOExpanderInterfaceInstance = etl::singleton<MCP23017Interface>;
#endif