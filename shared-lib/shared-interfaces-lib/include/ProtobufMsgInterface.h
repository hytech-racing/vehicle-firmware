#ifndef PROTOBUFMSGINTERFACE
#define PROTOBUFMSGINTERFACE

#include "circular_buffer.h"
#include "etl/optional.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include <QNEthernet.h>

template <size_t BufferSize, typename PbMsgType>
etl::optional<PbMsgType> handleEthernetSocketReceive(qindesign::network::EthernetUDP* socket, const pb_msgdesc_t* desc_pointer)
{
    int packet_size = socket->parsePacket();
    if (packet_size > 0)
    {
        uint8_t buffer[BufferSize];
        size_t read_bytes = socket->read(buffer, sizeof(buffer));
        socket->read(buffer, BufferSize); // remove?

        pb_istream_t stream = pb_istream_from_buffer(buffer, packet_size);
        PbMsgType msg = {};
        if (pb_decode(&stream, desc_pointer, &msg))
        {
            return msg;
        }
    }
    return etl::nullopt;
}

template <size_t BufferSize, typename PbStruct>
bool handleEthernetSocketSendPB(IPAddress addr, uint16_t port, qindesign::network::EthernetUDP* socket, const PbStruct& msg, const pb_msgdesc_t* msg_desc)
{
    socket->beginPacket(addr, port);
    uint8_t buffer[BufferSize];
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