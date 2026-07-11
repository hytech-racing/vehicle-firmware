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
                        qindesign::network::EthernetUDP *udp_socket);

    void receive_drivebrain_speed_command_telem(const CAN_message_t &msg, unsigned long curr_millis);

    void receive_drivebrain_torque_lim_command_telem(const CAN_message_t &msg, unsigned long curr_millis);

    void receive_drivebrain_speed_command_auxillary(const CAN_message_t &msg, unsigned long curr_millis);

    void receive_drivebrain_torque_lim_command_auxillary(const CAN_message_t &msg, unsigned long curr_millis);

    void handle_enqueue_suspension_CAN_data(const ADCInterface &adc_instance);

    void handle_enqueue_coolant_temp_CAN_data(const ADCInterface &adc_instance);

    void handle_enqueue_flowmeter_CAN_data(FlowmeterInterface &flowmeter_instance, unsigned long curr_millis);

    void handle_send_ethernet_data(const hytech_msgs_VCRData_s &data);

    StampedDrivetrainCommand_s get_latest_telem_drivebrain_command();

    StampedDrivetrainCommand_s get_latest_auxillary_drivebrain_command();

private:

    IPAddress _drivebrain_ip;
    uint16_t _vcr_data_port;
    qindesign::network::EthernetUDP *_udp_socket;

    StampedDrivetrainCommand_s _latest_drivebrain_command_telem = {};
    StampedDrivetrainCommand_s _latest_drivebrain_command_auxillary = {};

};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif // DRIVEBRAININTERFACE_H
