#ifndef DRIVEBRAIN_INTERFACE_HPP
#define DRIVEBRAIN_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.h"
#include "ProtobufMsgInterface.h"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"
#include "hytech_msgs.pb.h"

#include <FlexCAN_T4.h>
#include <IPAddress.h>
#include <QNEthernet.h>
#include <cstdint>

/* Local Interface Includes */
#include "ADCInterface.hpp"
#include "FlowmeterInterface.hpp"


class DrivebrainInterface
{
  public:
    DrivebrainInterface(IPAddress drivebrain_ip, uint16_t vcr_data_port, qindesign::network::EthernetUDP* udp_socket);

    void receiveDrivebrainSpeedCommandTelem(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveDrivebrainTorqueLimCommandTelem(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveDrivebrainSpeedCommandAuxillary(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveDrivebrainTorqueLimCommandAuxillary(const CAN_message_t& msg, unsigned long curr_millis);

    void handleEnqueueSuspensionCANData(const ADCInterface& adc_instance);

    void handleEnqueueCoolantTempCANData(const ADCInterface& adc_instance);

    void handleEnqueueFlowmeterCANData(FlowmeterInterface& flowmeter_instance, unsigned long curr_millis);

    void handleSendEthernetData(const hytech_msgs_VCRData_s& data);

    StampedDrivetrainCommand_s getLatestTelemDrivebrainCommand();

    StampedDrivetrainCommand_s getLatestAuxillaryDrivebrainCommand();

  private:
    IPAddress _drivebrain_ip;
    uint16_t _vcr_data_port;
    qindesign::network::EthernetUDP* _udp_socket;

    StampedDrivetrainCommand_s _latest_drivebrain_command_telem = {};
    StampedDrivetrainCommand_s _latest_drivebrain_command_auxillary = {};
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif // DRIVEBRAININTERFACE_H
