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

class CCUEthernetInterface {
    public:
    CCUEthernetInterface();

    void init_ethernet_device();

    void receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data);
    
    private:
    EthernetUDP _ccu_data_recv_socket;

}

#endif