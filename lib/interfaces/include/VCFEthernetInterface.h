#ifndef VCF_ETHERNET_INTERFACE_H
#define VCF_ETHERNET_INTERFACE_H

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "EthernetAddressDefs.h"
#include <QNEthernet.h>
#include "ht_can_version.h"
#include "device_fw_version.h"

/* Local Interface Includes */
#include "ADCInterface.h"
#include "DashboardInterface.h"
#include "PedalsSystem.h"
#include "BrakeRotorTempInterface.h"
#include "SteeringSystem.h"

using namespace qindesign::network;


class VCFEthernetInterface
{
public:

    VCFEthernetInterface() = default;

    void init_ethernet_device();

    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCFData_s.
     *
     * @param shared_state The current VCF state, which includes both interface and system data.
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCFData_s make_vcf_data_msg(ADCInterface &adc_int,
                                            DashboardInterface &dash_int,
                                            PedalsSystem &pedals_sys,
                                            SteeringSystem &steering_sys,
                                            BrakeRotorTempInterface &brake_rotor_temp_int
    );

    /**
     * Function to take a populated protoc struct from VCR and update the VCF state. This is ONLY critical
     * for buzzer control!
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCF state.
     *
     * @post After this function completes, shared_state will contain the updated buzzer control.
     */
    void receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, VCFData_s &shared_state, unsigned long curr_millis);

    void handle_send_ethernet_vcf_data(const hytech_msgs_VCFData_s &data);

private:

    static constexpr size_t VER_HASH_LEN = 9;
    static constexpr float SHDN_HIGH_THRESHOLD = 12.0f; // threshold for shutdown being considered high

    /* Ethernet Sockets */
    EthernetUDP _vcf_send_socket;
    EthernetUDP _vcr_recv_socket;
};

using VCFEthernetInterfaceInstance = etl::singleton<VCFEthernetInterface>;

#endif /* VCF_ETHERNET_INTERFACE_H */