#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"
#include "hytech_msgs.pb.h"



#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"
#include "etl/optional.h"

using namespace qindesign::network;
EthernetUDP socket; 

void init_ethernet_device()
{
    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
    //recv_socket.begin(5555);
}

void test_send()
{
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(ADCInterfaceInstance::instance(), DashboardInterfaceInstance::instance(), PedalsSystemInstance::instance());
    if (handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().VCRData_port, &socket, msg, &hytech_msgs_VCFData_s_msg)) {
        Serial.println("Sent");
    } else {
        Serial.println("Failed");
    }
}

void test_receive()
{
    //handle_ethernet_socket_receive
    //curr_millis, socket(recv), msg desc, sizeof buffer, ref to empty protoc struct
    //return optional struct
}

void setup()
{
    init_ethernet_device();
}

void loop()
{
    etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&socket, &hytech_msgs_VCRData_s_msg);
    if (protoc_struct)
    {
        Serial.printf("message RR: %d\n", (*protoc_struct).rear_loadcell_data.RR_loadcell_analog);
        Serial.printf("message RL: %d\n", (*protoc_struct).rear_loadcell_data.RL_loadcell_analog);
        
    } 


    //test_send();
    //Serial.println("loopin");
}