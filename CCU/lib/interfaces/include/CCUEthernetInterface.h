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

#include "ACUInterface.h"

using namespace qindesign::network;

class CCUEthernetInterface {
    public:
    CCUEthernetInterface();

    void initEthernetDevice();

    void receiveACUAllData(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data);

    hytech_msgs_CCUData makeCCUDatamsg();

    void sendCCUDataMsg(const hytech_msgs_CCUData &data);
    
    private:
    EthernetUDP _ccu_data_recv_socket;
    EthernetUDP _ccu_data_send_socket;
}

using CCUEthernetInterfaceInstance = etl::singleton<CCUEthernetInterfaceInstance>;

#endif