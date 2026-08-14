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

    /**
     * We want to build this class like an abstract interface class, something you wil learn in CS1331. When building an abstract
     * interface class in C++, we always want to use virtual,
     *
     * Virtual: The main keyword for polymorphism in C++. Without virtual, C++ decides which function to call based purely on the
     *          static type rather the runtime type.
     *
     * Destructor: A special member function in C++ that runs automatically when an object is destroyed. Syntax: ~ClassName
     *             It's job is to release any resources the object acquired during its lifetime.
    */
    virtual ~IOExpanderInterface() = default;

    virtual void read() = 0;

    virtual bool get_bit_port_a(uint8_t bit) = 0;

    virtual bool get_bit_port_b(uint8_t bit) = 0;

};
#endif
