#ifndef PROTOBUFMSGINTERFACE
#define PROTOBUFMSGINTERFACE

#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"

#include <QNEthernet.h>

#include "circular_buffer.h"
#include "etl/optional.h"

template <size_t buffer_size, typename pb_msg_type>
etl::optional<pb_msg_type> handle_ethernet_socket_receive(qindesign::network::EthernetUDP *socket, const pb_msgdesc_t *desc_pointer)
{
    int packet_size = socket->parsePacket();
    if (packet_size > 0)
        {
        uint8_t buffer[buffer_size];
        size_t read_bytes = socket->read(buffer, sizeof(buffer));
        socket->read(buffer, buffer_size); //remove?

        pb_istream_t stream = pb_istream_from_buffer(buffer, packet_size);
        pb_msg_type msg = {};
        if (pb_decode(&stream, desc_pointer, &msg))
        {
            return msg;
        }
    }
    return etl::nullopt;
}

template <size_t buffer_size, typename pb_struct>
bool handle_ethernet_socket_send_pb(IPAddress addr, uint16_t port, qindesign::network::EthernetUDP *socket, const pb_struct &msg, const pb_msgdesc_t *msg_desc)
{
    socket->beginPacket(addr, port);
    uint8_t buffer[buffer_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (!pb_encode(&stream, msg_desc, &msg))
    {
        // You can handle error more explicitly by looking at stream.errmsg
        return false;
    }
    auto message_length = stream.bytes_written;
    socket->write(buffer, message_length);
    socket->endPacket();
    return true;
}



#endif