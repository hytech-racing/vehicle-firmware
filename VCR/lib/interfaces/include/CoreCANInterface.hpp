#ifndef CORE_CAN_INTERFACE_HPP
#define CORE_CAN_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.hpp"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "DrivebrainInterface.hpp"
#include "InverterInterface.hpp"
#include "RSSInterface.hpp"
#include "VCFInterface.hpp"


/**
 * @brief The CAN buses on VCR. Used to select which bus a message is enqueued on.
 *
 * @note TODO: rear aux (CAN2) is not here because it was never started (its setup was commented out
 * before the move to CANBus). Constructing a CANBus starts the hardware, so add REAR_AUX and a
 * CANBus<CAN2> member when it is ready to be brought up.
 */
enum class VCRCANBusses_e
{
    TELEM,
    INVERTER
};

/**
 * @brief Owns VCR's CAN buses. This is the only code with access to them: everything else sends by
 * calling enqueueSwitch() / packAndEnqueueOnSwitch(), and received messages are routed to the
 * interfaces passed in on construction.
 */
class VCRCANInterface : public CANInterfaceTemplate_t<VCRCANBusses_e>
{
  public:
    VCRCANInterface(
        ACUInterface& acu_interface,
        DrivebrainInterface& db_interface,
        InverterInterface& fl_inverter_interface,
        InverterInterface& fr_inverter_interface,
        InverterInterface& rl_inverter_interface,
        InverterInterface& rr_inverter_interface,
        VCFInterface& vcf_interface,
        RSSInterface& rss_interface,
        uint32_t telem_baudrate,
        uint32_t inverter_baudrate)
        : _acu_interface(acu_interface),
          _db_interface(db_interface),
          _fl_inverter_interface(fl_inverter_interface),
          _fr_inverter_interface(fr_inverter_interface),
          _rl_inverter_interface(rl_inverter_interface),
          _rr_inverter_interface(rr_inverter_interface),
          _vcf_interface(vcf_interface),
          _rss_interface(rss_interface),
          _telem_bus(telem_baudrate),
          _inverter_bus(inverter_baudrate)
    {
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueueSwitch(VCRCANBusses_e bus, const CAN_message_t& msg) override;

    /**
     * @brief Drains every RX buffer, routing each message to its interface
     */
    void processAllRxBuffers() override;

    /**
     * @brief Writes every message in every TX buffer out onto its bus
     */
    void sendAllTxBuffers() override;

  private:
    ACUInterface& _acu_interface;
    DrivebrainInterface& _db_interface;
    InverterInterface& _fl_inverter_interface;
    InverterInterface& _fr_inverter_interface;
    InverterInterface& _rl_inverter_interface;
    InverterInterface& _rr_inverter_interface;
    VCFInterface& _vcf_interface;
    RSSInterface& _rss_interface;

    CANBus<CAN1> _telem_bus;
    CANBus<CAN3> _inverter_bus;

    /**
     * @brief Per-bus receive handlers. They only tag the message with its bus, since the drivebrain
     * messages are routed differently depending on which bus they arrived on.
     */
    void _onTelemBusRecv(const CAN_message_t& msg);
    void _onInverterBusRecv(const CAN_message_t& msg);

    /**
     * @brief Routes a received message to the appropriate interface based on CANID
     */
    void _recvSwitch(const CAN_message_t& msg, VCRCANBusses_e bus);
};

using VCRCANInterfaceInstance = etl::singleton<VCRCANInterface>;

#endif // CORE_CAN_INTERFACE_HPP
