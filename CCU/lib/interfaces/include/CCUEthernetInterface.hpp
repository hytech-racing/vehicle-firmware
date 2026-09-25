#ifndef CCUETHERNETINTERFACE_H
#define CCUETHERNETINTERFACE_H

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "EthernetAddressDefs.h"
#include <QNEthernet.h>
#include <algorithm>
#include <cstddef>
#include <iterator>

using namespace qindesign::network;


/**
 * @brief Output values from CCU to ACU.
 */
struct CCUOutput_s
{
    float current_setpoint;
    bool allow_balance;
};

namespace CCUEthernetInterface
{
    void recieve_pb_msg_acu_all_data(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data);
};

// class CCUEthernettInterface
// {
// public:
//     void init_ethernet_device();

//     void handle_recv_ethernet_acu_all_data(const hytech_msgs_ACUAllData &data);

//     void handle_recv_ethernet_acu_all_data(const hytech_msgs_ACUAllData &data);

// private:
//     /* Ethernet Sockets */
//     EthernetUDP _acu_core_data_recv_socket;
//     EthernetUDP _acu_all_data_recv_socket;
// }


#endif