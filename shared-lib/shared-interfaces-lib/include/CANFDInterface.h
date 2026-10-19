#ifndef CANFDINTERFACE
#define CANFDINTERFACE

#include <cstdint>
#include <cstring>
#include "FlexCAN_T4.h"

#include "etl/delegate.h"

#include "CANInterface.h"

/**
 * @brief CAN setup function for flexcan devices
 *
 * @tparam CAN_DEVICE the type of FD CAN device being used 
 * @param CAN_dev ref to the CAN device to be setup
 * @param timings the CAN FD timings to set the device to have
 * @param on_recv_func the receive function callback that will get registered to be ran on the receive of a CAN message
 * *IMPORTANT* due to the way setBaudRate works (setting clock for all CAN),
 * you MUST run handle_CANFD_setup BEFORE handle_CAN_setup
 */
template <typename CAN_DEVICE>
bool handle_CANFD_setup(CAN_DEVICE &CAN_dev, const CANFD_timings_t &timings, void (*on_recv_func)(const CANFD_message_t &msg))
{
    CAN_dev.begin();
    bool set_baud = CAN_dev.setBaudRate(timings); // returns false if no valid timing for the selected clock
    uint8_t num_mb = CAN_dev.setRegions(64);
    uint8_t num_rx_mb = 10;
    for (uint8_t i=0; i<num_mb; i++)
    {
         CAN_dev.setMB(static_cast<FLEXCAN_MAILBOX>(i), i < num_rx_mb ? RX : TX, STD);
    }
    CAN_dev.enableMBInterrupts();
    CAN_dev.onReceive(on_recv_func);
    return set_baud;
}

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
void process_ring_buffer_FD(BufferType &rx_buffer, InterfaceContainer &interfaces, unsigned long curr_millis, etl::delegate<void(InterfaceContainer& interfaces, const CANFD_message_t& CAN_msg, unsigned long curr_millis, CANInterfaceType_e interface_type)> recv_switch_func, CANInterfaceType_e interface_type)
{
    while (rx_buffer.available())
    {
        CANFD_message_t recvd_msg;
        uint8_t buf[sizeof(CANFD_message_t)];
        rx_buffer.pop_front(buf, sizeof(CANFD_message_t));
        memmove(&recvd_msg, buf, sizeof(recvd_msg));
        recv_switch_func(interfaces, recvd_msg, curr_millis, interface_type);
    }
}

namespace CAN_util
{
    template <typename can_struct, typename queue_type>
    void enqueue_fd_msg(can_struct* structure, uint32_t (* pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*), queue_type& CAN_msg_out_queue) {
        CANFD_message_t can_msg;
        can_msg.id = pack_function(structure, can_msg.buf, &can_msg.len, (uint8_t*) &can_msg.flags.extended);
        can_msg.edl = 1; // set EDL flag to indicate CAN FD message
        can_msg.brs = 1; // set BRS flag to indicate baud rate switching for data
        uint8_t buf[sizeof(CANFD_message_t)] = {};
        memmove(buf, &can_msg, sizeof(CANFD_message_t));
        CAN_msg_out_queue.push_back(buf, sizeof(CANFD_message_t));
    }

    template <typename can_struct, typename queue_type>
    void enqueue_fd_msg(can_struct* structure, uint32_t (* pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*), queue_type& CAN_msg_out_queue, uint32_t id) {
        CANFD_message_t can_msg;
        pack_function(structure, can_msg.buf, &can_msg.len, (uint8_t*) &can_msg.flags.extended);
        can_msg.id = id;
        can_msg.edl = 1; // set EDL flag to indicate CAN FD message
        can_msg.brs = 1; // set BRS flag to indicate baud rate switching for data
        uint8_t buf[sizeof(CANFD_message_t)] = {};
        memmove(buf, &can_msg, sizeof(CANFD_message_t));
        CAN_msg_out_queue.push_back(buf, sizeof(CANFD_message_t));
    }
}

#endif /* CANFDINTERFACE */