#ifndef CORE_CAN_INTERFACE_HPP
#define CORE_CAN_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.hpp"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

/* Local Interface Includes */
#include "DrivebrainInterface.hpp"
#include "FDCInterface.hpp"
#include "RSSInterface.hpp"


/**
 * @brief The CAN buses on RDC. Used to select which bus a message is enqueued on.
 */
enum class RDCCANBusses_e
{
    DRIVERLESS
};

/**
 * @brief Owns RDC's CAN bus. This is the only code with access to it: everything else sends by
 * calling enqueueOnSwitch() / enqueueCustomIDOnSwitch() with an already-packed message, and received messages are routed to the
 * interfaces passed in on construction.
 */
class RDCCANInterface : public CANInterface<RDCCANBusses_e>
{
  public:
    RDCCANInterface(DrivebrainInterface& db_interface, FDCInterface& fdc_interface, RSSInterface& rss_interface, uint32_t driverless_baudrate)
        : _db_interface(db_interface), _fdc_interface(fdc_interface), _rss_interface(rss_interface), _driverless_bus(driverless_baudrate)
    {
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueueOnSwitch(RDCCANBusses_e bus, const CAN_message_t& msg) override;

    /**
     * @brief Drains the RX buffer, routing each message to its interface
     */
    void processAllRxBuffers() override;

    /**
     * @brief Writes every message in the TX buffer out onto the bus
     */
    void sendAllTxBuffers() override;

  private:
    DrivebrainInterface& _db_interface;
    FDCInterface& _fdc_interface;
    RSSInterface& _rss_interface;

    CANBus<CAN1> _driverless_bus;

    /**
     * @brief Routes a received message to the appropriate interface based on CANID
     */
    void _recvSwitch(const CAN_message_t& msg);
};

using RDCCANInterfaceInstance = etl::singleton<RDCCANInterface>;

#endif // CORE_CAN_INTERFACE_HPP
