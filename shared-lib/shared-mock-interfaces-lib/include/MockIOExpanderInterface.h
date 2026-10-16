#ifndef MOCK_IOEXPANDER_INTERFACE_H
#define MOCK_IOEXPANDER_INTERFACE_H
#include "IOExpanderInterface.h"
#include <gmock/gmock.h>

class MockIOExpanderInterface : public IOExpanderInterface
{
public:

    MOCK_METHOD(void, read, (), (override));

    MOCK_METHOD(bool, getBitPortA, (uint8_t), (override));

    MOCK_METHOD(bool, getBitPortB, (uint8_t), (override));

};

#endif