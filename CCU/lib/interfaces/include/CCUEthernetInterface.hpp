#ifndef CCUETHERNETINTERFACE_H_
#define CCUETHERNETINTERFACE_H_

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "EthernetAddressDefs.h"
#include <QNEthernet.h>
#include <algorithm>
#include <cstddef>
#include <iterator>

/* ETL Library */
#include <etl/singleton.h>


#include "ACUInterface.h"
#include "ChargerInterface.h"
#include "EMInterface.h"
#include "MainChargeSystem.h"

using namespace qindesign::network;

namespace ccu_ethernet_params
{
  constexpr const uint8_t NUM_CELLS = 126;
  constexpr const uint8_t NUM_CELLTEMPS = 48;
  constexpr const uint8_t NUM_CHIPS = 12;
};

struct CCUParams_s
{
  uint8_t num_cells;
  uint8_t num_celltemps;
  uint8_t num_chips;
};

class CCUEthernetInterface {
    public:
    CCUEthernetInterface(CCUParams_s params = {
                                .num_cells = ccu_ethernet_params::NUM_CELLS,
                                .num_celltemps = ccu_ethernet_params::NUM_CELLTEMPS,
                                .num_chips = ccu_ethernet_params::NUM_CHIPS
                            }
    ) :_ccu_params(params) 
    {};

    void initEthernetDevice();

    void receiveACUAllData(const hytech_msgs_ACUAllData &msg_in, ACUAllDataType_s &acu_all_data);

    hytech_msgs_CCUData makeCCUDataMsg();

    void sendCCUDataMsg(const hytech_msgs_CCUData &data);
    void sendCCUTest(void);
    
    private:
    const CCUParams_s _ccu_params = {};
    EthernetUDP _acu_all_data_recv_socket;
    EthernetUDP _ccu_data_send_socket;
};

using CCUEthernetInterfaceInstance = etl::singleton<CCUEthernetInterface>;

#endif // CCUETHERNETINTERFACE_H_