#ifndef CANINTERFACE
#define CANINTERFACE

#include <cstdint>
#include "FlexCAN_T4.h"

#include "etl/delegate.h"


/**
 * @brief CAN setup function for flexcan devices
 *
 * @tparam CAN_DEVICE the type of the CAN device being used (flexcan has all sorts of template args im not gonna call out here)
 * @param CAN_dev ref to the CAN device to be setup
 * @param baudrate the baudrate to set the device to have
 * @param on_recv_func the receive function callback that will get registered to be ran on the receive of a CAN message
 */
template <typename CAN_DEVICE>
void handle_CAN_setup(CAN_DEVICE& CAN_dev, uint32_t baudrate, void (*on_recv_func)(const CAN_message_t &msg))
{
    CAN_dev.begin();
    CAN_dev.setBaudRate(baudrate);
    CAN_dev.setMaxMB(16);
    CAN_dev.enableFIFO();
    CAN_dev.enableFIFOInterrupt();
    CAN_dev.onReceive(on_recv_func);
}

// reads from receive buffer updating the current message frame from a specific receive buffer
// TODO ensure that all of the repeated interfaces are at the correct IDs
/*
 Reads from the specified receive buffer and passes through messages to
 the callback associated with the CAN message ID.
*/

/**
 * @enum CanInterfaceType_e
 *
 * Provides the different can interfaces we use
 * so that they can be referenced in generic methods
 * such as process_ring_buffer
 */
enum CANInterfaceType_e
{
    ACU,
    CCU,
    CHARGER,
    EM,
    FAUX,
    RAUX,
    TELEM,
    INVERTER
};

/**
 * @brief handles reading from a receive buffer updating the current message frame from a specific receive buffer. pass through messages to the callback specified using the delegate function
 *
 * @tparam BufferType CAN message receive buffer type (::pop_front(buf, len))
 * @tparam InterfaceContainer the type of struct holding refs / pointers to CAN interfaces that have receive callbacks
 * @param rx_buffer ref to receive buffer being received from
 * @param interfaces ref to interfaces struct (passed to the recv_switch_function)
 * @param curr_millis current millis
 * @param recv_switch_func the receive function that gets called and is given the interfaces ref, CAN message struct and millis timestamp. expected to contain switch statement.
 */
template <typename BufferType, typename InterfaceContainer>
void process_ring_buffer(BufferType &rx_buffer, InterfaceContainer &interfaces, unsigned long curr_millis, etl::delegate<void(InterfaceContainer& interfaces, const CAN_message_t& CAN_msg, unsigned long curr_millis, CANInterfaceType_e interface_type)> recv_switch_func, CANInterfaceType_e interface_type)
{
    while (rx_buffer.available())
    {
        CAN_message_t recvd_msg;
        uint8_t buf[sizeof(CAN_message_t)];
        rx_buffer.pop_front(buf, sizeof(CAN_message_t));
        memmove(&recvd_msg, buf, sizeof(recvd_msg));
        recv_switch_func(interfaces, recvd_msg, curr_millis, interface_type);
    }
}

namespace CAN_util
{
    template <typename can_struct, typename queue_type>
    void enqueue_msg(can_struct* structure, uint32_t (* pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*), queue_type& CAN_msg_out_queue) {
        CAN_message_t can_msg;
        can_msg.id = pack_function(structure, can_msg.buf, &can_msg.len, (uint8_t*) &can_msg.flags.extended);
        uint8_t buf[sizeof(CAN_message_t)] = {};
        memmove(buf, &can_msg, sizeof(CAN_message_t));
        CAN_msg_out_queue.push_back(buf, sizeof(CAN_message_t));
    }

    template <typename can_struct, typename queue_type>
    void enqueue_msg(can_struct* structure, uint32_t (* pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*), queue_type& CAN_msg_out_queue, uint32_t id) {
        CAN_message_t can_msg;
        pack_function(structure, can_msg.buf, &can_msg.len, (uint8_t*) &can_msg.flags.extended);
        can_msg.id = id;
        uint8_t buf[sizeof(CAN_message_t)] = {};
        memmove(buf, &can_msg, sizeof(CAN_message_t));
        CAN_msg_out_queue.push_back(buf, sizeof(CAN_message_t));
    }

}

#endif /* CANINTERFACE */
