#ifndef VCFCANINTERFACE_H
#define VCFCANINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.h"
#include "SharedFirmwareTypes.h"

#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "BrakeRotorTempInterface.hpp"
#include "DashboardInterface.hpp"
#include "VCRInterface.hpp"


/**
 * @brief The CAN buses on VCF. Used to select which bus a message is enqueued on.
 */
enum class VCFCANBus_e
{
    TELEM,
    FRONT_AUX
};

/**
 * @brief Owns VCF's CAN peripherals and RX/TX buffers. This is the only code with access to them:
 * everything else sends by calling enqueue(), and received messages are routed to the interfaces
 * passed in on construction.
 */
class VCFCANInterface
{
  public:
    VCFCANInterface(
        ACUInterface& acu_interface,
        BrakeRotorTempInterface& brake_rotor_temp_interface,
        DashboardInterface& dash_interface,
        VCRInterface& vcr_interface)
        : _acu_interface(acu_interface),
          _brake_rotor_temp_interface(brake_rotor_temp_interface),
          _dash_interface(dash_interface),
          _vcr_interface(vcr_interface)
    {
    }

    /**
     * @brief Sets up both CAN peripherals and registers their receive callbacks
     */
    void init(uint32_t telem_baudrate, uint32_t front_aux_baudrate);

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueue(VCFCANBus_e bus, const CAN_message_t& msg);

    /**
     * @brief Packs a CAN struct and enqueues it onto the given bus' TX buffer
     * @param msg the unpacked CAN struct (e.g. PEDALS_SYSTEM_DATA_t)
     * @param pack_function the generated pack function (e.g. Pack_PEDALS_SYSTEM_DATA_ht_can)
     */
    template <typename can_struct>
    void enqueue(VCFCANBus_e bus, can_struct* msg, uint32_t (*pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*))
    {
        CAN_message_t can_msg;
        can_msg.id = pack_function(msg, can_msg.buf, &can_msg.len, (uint8_t*)&can_msg.flags.extended); // NOLINT
        enqueue(bus, can_msg);
    }

    /**
     * @brief Drains both RX buffers, routing each message to its interface
     */
    void processAllRxBuffers(unsigned long curr_millis);

    /**
     * @brief Writes every message in both TX buffers out onto its bus
     */
    void sendAllTxBuffers();

  private:
    static constexpr size_t _CAN_MSG_SIZE = sizeof(CAN_message_t);
    using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, _CAN_MSG_SIZE>;
    using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, _CAN_MSG_SIZE>;

    template <CAN_DEV_TABLE CAN_DEV>
    using FlexCAN_t = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

    /* Interfaces that received messages are routed to */
    ACUInterface& _acu_interface;
    BrakeRotorTempInterface& _brake_rotor_temp_interface;
    DashboardInterface& _dash_interface;
    VCRInterface& _vcr_interface;

    /* CAN peripherals and buffers */
    FlexCAN_t<CAN1> _telem_can;
    CANRXBuffer_t _telem_can_rx_buffer;
    CANTXBuffer_t _telem_can_tx_buffer;

    FlexCAN_t<CAN2> _front_aux_can;
    CANRXBuffer_t _front_aux_can_rx_buffer;
    CANTXBuffer_t _front_aux_can_tx_buffer;

    /**
     * @brief FlexCAN receive callbacks. Static because FlexCAN only accepts plain function pointers;
     * they push into the singleton instance's RX buffers.
     */
    static void _onTelemCANRecv(const CAN_message_t& msg);
    static void _onFrontAuxCANRecv(const CAN_message_t& msg);

    static void _pushMsg(CANRXBuffer_t& buffer, const CAN_message_t& msg); // static: called from the static callbacks above
    void _sendAllCANMsgs(CANTXBuffer_t& buffer, FlexCAN_T4_Base& can_interface);

    void _processRxBuffer(CANRXBuffer_t& buffer, unsigned long curr_millis, CANInterfaceType_e interface_type);

    /**
     * @brief Routes a received message to the appropriate interface based on CANID
     */
    void _recvSwitch(const CAN_message_t& msg, unsigned long curr_millis, CANInterfaceType_e interface_type);
};

using VCFCANInterfaceInstance = etl::singleton<VCFCANInterface>;

#endif // VCFCANINTERFACE_H
