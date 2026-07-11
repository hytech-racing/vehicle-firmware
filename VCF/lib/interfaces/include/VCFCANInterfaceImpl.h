#ifndef VCFCANINTERFACEIMPL_H
#define VCFCANINTERFACEIMPL_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "ACUInterface.h"
#include "BrakeRotorTempInterface.h"
#include "DashboardInterface.h"
#include "VCRInterface.h"

/* Globally accessible types */
constexpr size_t CAN_MSG_SIZE = sizeof(CAN_message_t);
using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CAN_MSG_SIZE>;
using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CAN_MSG_SIZE>;

template <CAN_DEV_TABLE CAN_DEV>
using FlexCAN_t = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;


/**
 * @brief This struct holds references to the interface objects that use decoded CAN messages. References only!
 */
struct CANInterfaces_s
{
    explicit CANInterfaces_s(ACUInterface &acu_int,
                            BrakeRotorTempInterface &brake_rotor_temp_int,
                            DashboardInterface &dash_int,
                            VCRInterface &vcr_int
    ) : acu_interface(acu_int),
        brake_rotor_temp_interface(brake_rotor_temp_int),
        dash_interface(dash_int),
        vcr_interface(vcr_int)
    {};

    ACUInterface &acu_interface;
    BrakeRotorTempInterface &brake_rotor_temp_interface;
    DashboardInterface &dash_interface;
    VCRInterface &vcr_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

/**
 * @brief This struct holds the FlexCAN peripheral instances and their associated RX/TX ring buffers.
 */
struct VCFCANInterface_s
{
    explicit VCFCANInterface_s(etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> recv_switch_func) : can_recv_switch(recv_switch_func) {}

    FlexCAN_t<CAN1> TELEM_CAN;
    CANRXBuffer_t telem_can_rx_buffer;
    CANTXBuffer_t telem_can_tx_buffer;

    FlexCAN_t<CAN2> FRONT_AUX_CAN;
    CANRXBuffer_t front_aux_can_rx_buffer;
    CANTXBuffer_t front_aux_can_tx_buffer;

    etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> can_recv_switch;
};
using VCFCANInterfaceInstance = etl::singleton<VCFCANInterface_s>;

namespace VCFCANInterfaceImpl
{
    void on_telem_can_recv(const CAN_message_t &msg);

    void on_front_aux_can_recv(const CAN_message_t &msg);

    /**
     * @brief Routes a decoded message to the appropriate interface based on CANID
     */
    void vcf_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
}

#endif // VCFCANINTERFACEIMPL_H
