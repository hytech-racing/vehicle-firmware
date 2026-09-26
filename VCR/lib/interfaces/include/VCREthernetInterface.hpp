#ifndef VCR_ETHERNET_INTERFACE
#define VCR_ETHERNET_INTERFACE

#include "controls.hpp"

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
#include "ADCInterface.hpp"
#include "InverterInterface.hpp"
#include "MCP23017Interface.hpp"
#include "VCFInterface.hpp"

/* Local System Includes */
#include "VehicleStateMachine.hpp"
#include "DrivetrainSystem.hpp"

using namespace qindesign::network;


class VCREthernetInterface
{
public:

    VCREthernetInterface() = default;

    void initEthernetDevice();

    /**
     * @brief Method to fill the protoc struct hytech_msgs_VCRData_s
     * @return A populated instance of the outgoing protoc struct
    */
    hytech_msgs_VCRData_s makeVCRDataPBMsg(const ADCInterface &adc_interface,
                                        DrivetrainDynamicReport_s &drivetrain_data,
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
    void receiveDrivebrainPBMsg(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Function to take a populated protoc struct from VCF and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receiveVCFPBMsg(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    EthernetUDP* get_vcr_data_send_socket() { return &_vcr_data_send_socket; }

private:

    EthernetUDP _vcr_data_send_socket;
    EthernetUDP _vcf_data_recv_socket;

    /**
     * @brief Method to copy an instance of InverterData_s to the protoc struct hytech_msgs_InverterData_s
     * @param original A populated instance of the InverterData_s
     * @param destination The destination protoc struct
     * @post The destination struct will be populated with the data from 'original'
    */
    void _copyInverterData(const InverterData_s &original, hytech_msgs_InverterData_s &destination);

     /**
     * @brief Method to copy an instance of InverterLimits_s to the protoc struct hytech_msgs_InverterLimits_s
     * @param original A populated instance of the InverterLimits_s defined in shared firmware types
     * @param destination The destination protoc struct
     * @post The destination struct will be populated with the data from 'original'
    */
    void _copyInverterLimits(const InverterLimits_s &original, hytech_msgs_InverterLimits_s &destination);

    /**
     * Helper function to copy veh_vec data.
     *
     * @param original A populated instance of a veh_vec.
     * @param destination A reference to an unpopulated instance of veh_vec.
     * @post The destination veh_vec will be populated with the data from the original.
    */
    template <typename from_T, typename to_T>
    void _copyVehVecMembers(const from_T& from, to_T& to)
    {
        to.FL = from.FL;
        to.FR = from.FR;
        to.RL = from.RL;
        to.RR = from.RR;
    };

};

using VCREthernetInterfaceInstance = etl::singleton<VCREthernetInterface>;

#endif /* VCR_ETHERNET_INTERFACE_H */