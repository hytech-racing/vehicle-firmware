#ifndef DRIVEBRAININTERFACE_H
#define DRIVEBRAININTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "CANInterface.h"
#include "hytech.h"
#include <FlexCAN_T4.h>
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include <QNEthernet.h>
#include <IPAddress.h>
#include <cstdint>

/* Local Interface Includes */
#include "ADCInterface.h"
#include "FlowmeterInterface.h"


class DrivebrainInterface
{
public:

    DrivebrainInterface(IPAddress drivebrain_ip,
                        uint16_t vcr_data_port,
                        qindesign::network::EthernetUDP *udp_socket
    ) : _drivebrain_ip(drivebrain_ip),
        _vcr_data_port(vcr_data_port),
        _udp_socket(udp_socket)
    {};

    void receiveDrivebrainSpeedCommandTELEM(const CAN_message_t &msg, unsigned long curr_millis);
    void receiveDrivebrainTorqueCommandTELEM(const CAN_message_t &msg, unsigned long curr_millis);

    void receiveDrivebrainSpeedCommandRAUX(const CAN_message_t &msg, unsigned long curr_millis);
    void receiveDrivebrainTorqueCommandRAUX(const CAN_message_t &msg, unsigned long curr_millis);

    void handleEnqueueSuspensionCANData(const ADCInterface &adc_instance);
    void handleEnqueueCoolantTempCANData(const ADCInterface &adc_instance);
    void handleEnqueueFlowmeterCANData(FlowmeterInterface &flowmeter_instance, unsigned long curr_millis);

    void handleSendEthernetData(const hytech_msgs_VCRData_s &data);

    StampedDrivetrainCommand_s getLatestDrivebrainCommandTELEM();

    StampedDrivetrainCommand_s getLatestDrivebrainCommandRAUX();

private:

    IPAddress _drivebrain_ip;
    uint16_t _vcr_data_port;
    qindesign::network::EthernetUDP *_udp_socket;

    StampedDrivetrainCommand_s _latest_drivebrain_command_telem = {};
    StampedDrivetrainCommand_s _latest_drivebrain_command_raux = {};

};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif // DRIVEBRAININTERFACE_H
