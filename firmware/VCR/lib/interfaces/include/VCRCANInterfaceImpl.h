#ifndef VCRCANINTERFACEIMPL_H
#define VCRCANINTERFACEIMPL_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "DrivebrainInterface.h"
#include "VCFInterface.h"
#include "ACUInterface.h"
#include "InverterInterface.h"

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
                            DrivebrainInterface &db_int,
                            InverterInterface &fl_inv_int,
                            InverterInterface &fr_inv_int,
                            InverterInterface &rl_inv_int,
                            InverterInterface &rr_inv_int,
                            VCFInterface &vcf_int
    ) : acu_interface(acu_int),
        db_interface(db_int),
        fl_inverter_interface(fl_inv_int),
        fr_inverter_interface(fr_inv_int),
        rl_inverter_interface(rl_inv_int),
        rr_inverter_interface(rr_inv_int),
        vcf_interface(vcf_int)
    {};

    ACUInterface &acu_interface;
    DrivebrainInterface &db_interface;
    InverterInterface &fl_inverter_interface;
    InverterInterface &fr_inverter_interface;
    InverterInterface &rl_inverter_interface;
    InverterInterface &rr_inverter_interface;
    VCFInterface &vcf_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

struct VCRCANInterface_s
{
    explicit VCRCANInterface_s(etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_switch_func) : can_recv_switch(recv_switch_func) {}

    FlexCAN_t<CAN1> TELEM_CAN;
    CANRXBuffer_t telem_can_rx_buffer;
    CANTXBuffer_t telem_can_tx_buffer;

    FlexCAN_t<CAN2> REAR_AUX_CAN;
    CANRXBuffer_t rear_aux_can_rx_buffer;
    CANTXBuffer_t rear_aux_can_tx_buffer;

    FlexCAN_t<CAN3> INVERTER_CAN;
    CANRXBuffer_t inverter_can_rx_buffer;
    CANTXBuffer_t inverter_can_tx_buffer;


    etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> can_recv_switch;
};
using VCRCANInterfaceInstance = etl::singleton<VCRCANInterface_s>;

namespace VCRCANInterfaceImpl
{
    void on_auxillary_can_receive(const CAN_message_t &msg);

    void on_inverter_can_receive(const CAN_message_t &msg);

    void on_telem_can_receive(const CAN_message_t &msg);

    void vcr_CAN_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
}; // namespace VCRCANInterfaceImpl

#endif // VCRCANINTERFACEIMPL_H