#ifndef IOEXPANDERINTERFACE_H
#define IOEXPANDERINTERFACE_H
#include <cstdint>


struct IOExpanderPortMode_s
{
    uint8_t directions;
    uint8_t pullups;
    uint8_t inverted;
};

struct IOExpanderParams_s
{
    uint8_t i2c_address;
    IOExpanderPortMode_s port_a;
    IOExpanderPortMode_s port_b;
};

class IOExpanderInterface
{
public:

    virtual ~IOExpanderInterface() = default;

    virtual void read() = 0;

    virtual bool get_bit_port_a(uint8_t bit) = 0;

    virtual bool get_bit_port_b(uint8_t bit) = 0;

};
#endif
