#ifndef VCR_ETHERNET_INTERFACE_H
#define VCR_ETHERNET_INTERFACE_H

#include "controls.h"

/* External Includes */
#include <algorithm>
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "EthernetAddressDefs.h"
#include <QNEthernet.h>
#include "ht_can_version.h"
#include "device_fw_version.h"

/* Local Interface Includes */
#include "ADCInterface.h"
#include "InverterInterface.h"
#include "MCP23017IOExpanderInterface.h"
#include "VCFInterface.h"

/* Local System Includes */
#include "VehicleStateMachine.h"
#include "DrivetrainSystem.h"

using namespace qindesign::network;


class VCREthernetInterface
{
public:

    VCREthernetInterface() = default;

    void init_ethernet_device();

    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCRData_s.
     *
     * @param
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCRData_s make_vcr_data_msg(const ADCInterface &adc_interface,
                                        DrivetrainDynamicReport_s &DrivetrainData,
                                        const VCFInterface &vcf_interface,
                                        const VehicleStateMachine &vehicle_state_machine,
                                        const DrivetrainSystem &drivetrain_system,
                                        const InverterInterface &fl_inverter,
                                        const InverterInterface &fr_inverter,
                                        const InverterInterface &rl_inverter,
                                        const InverterInterface &rr_inverter,
                                        const VCRControls &vcr_controls
    );

    /**
     * Function to take a populated protoc struct from the drivebrain and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Function to take a populated protoc struct from VCF and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Helper function to copy an instance of InverterData_s to the protoc struct hytech_msgs_InverterData_s.
     * @param original A populated instance of the InverterData_s defined in shared firmware types.
     * @param destination The destination protoc struct.
     * @post The destination struct will be populated with the data from original.
     */
    void copy_inverter_data(const InverterFeedbackData_s &original, hytech_msgs_InverterData_s &destination);

    /**
     * Helper function to copy veh_vec data.
     *
     * @param original A populated instance of a veh_vec.
     * @param destination A reference to an unpopulated instance of veh_vec.
     * @post The destination veh_vec will be populated with the data from the original.
     */
    template <typename from_T, typename to_T>
    void copy_veh_vec_members(const from_T& from, to_T& to)
    {
        to.FL = from.FL;
        to.FR = from.FR;
        to.RL = from.RL;
        to.RR = from.RR;
    };

    EthernetUDP* get_vcr_data_send_socket() { return &vcr_data_send_socket; }

private:

    /* Ethernet Sockets */
    EthernetUDP vcr_data_send_socket;
    EthernetUDP vcf_data_recv_socket;

};

using VCREthernetInterfaceInstance = etl::singleton<VCREthernetInterface>;

#endif /* VCR_ETHERNET_INTERFACE_H */