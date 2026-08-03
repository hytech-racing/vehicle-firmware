#include <Arduino.h>

#include "CANInterface.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> INV_CAN;

Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN2_rxBuffer;

void on_can2_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN2_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

struct CANInterfaces
{
    int test;
} asdf;

void recv_func(CANInterfaces& interfaces, uint32_t id, unsigned long millis)
{
    switch(id)
    {
        case 3:
        {
            break;
        } 
        default:
        {
            break;
        }
        
    }
}
auto recv_call = etl::delegate<void(CANInterfaces&, uint32_t, unsigned long)>::create<recv_func>();

void setup()
{
    handle_CAN_setup(INV_CAN, 500000, on_can2_receive);
}



void loop()
{
    process_ring_buffer(CAN2_rxBuffer, asdf, millis(), recv_call);
}