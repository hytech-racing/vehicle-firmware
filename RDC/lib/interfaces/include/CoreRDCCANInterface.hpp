#ifndef RDCCANINTERFACEIMPL_H
#define RDCCANINTERFACEIMPL_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.h"
#include "SharedFirmwareTypes.h"
#include <FlexCAN_T4.h>
#include <ht_can.h>

/* Local Interface Includes */
#include "DrivebrainInterface.hpp"
#include "FDCInterface.hpp"
#include "RSSInterface.hpp"

/* Globally accessible types */
using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;
using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;

/* CANBus types */
using DRIVERLESS_CAN_t = FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;

struct CANInterfaces_s
{
    explicit CANInterfaces_s(DrivebrainInterface &db_int, FDCInterface &fdc_int, RSSInterface &rss_int)
        : db_interface(db_int),
          fdc_interface(fdc_int),
          rss_interface(rss_int) {};

    DrivebrainInterface &db_interface;
    FDCInterface &fdc_interface;
    RSSInterface &rss_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

namespace RDCCANInterfaceImpl
{
    void RDCRecvSwitch(CANInterfaces_s &, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type);
    void onDVCANRecvHelper(const CAN_message_t &msg);
} // namespace RDCCANInterfaceImpl

class CoreRDCCANInterface
{
    public:
    CoreRDCCANInterface(etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recvSwitchFunction, uint32_t baudrate)
        : recvSwitch(recvSwitchFunction)
    {
        handle_CAN_setup(DRIVERLESS_CAN, baudrate, &RDCCANInterfaceImpl::onDVCANRecvHelper);
    }

    etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recvSwitch;

    void sendAllCanMsgs();
    void onDVCANRecv(const CAN_message_t &msg);

    template <typename CANStruct> void enqueueMsg(CANStruct *msg, uint32_t (*packFunction)(CANStruct *, uint8_t *, uint8_t *, uint8_t *))
    {
        CAN_util::enqueue_msg(msg, packFunction, _driverless_can_tx_buffer);
    }

    void processCANMessages(
        CANInterfaces_s can_interfaces,
        unsigned long curr_millis,
        etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recvSwitch,
        CANInterfaceType_e interface_type);

    private:
    DRIVERLESS_CAN_t DRIVERLESS_CAN;

    CANTXBuffer_t _driverless_can_tx_buffer;
    CANRXBuffer_t _driverless_can_rx_buffer;
};

using CoreRDCCANInterfaceInstance = etl::singleton<CoreRDCCANInterface>;
#endif
