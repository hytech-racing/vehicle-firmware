#ifndef CORE_CAN_INTERFACE_HPP
#define CORE_CAN_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.hpp"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

/* Local Interface Includes */
#include "CCUInterface.hpp"
#include "EMInterface.hpp"


/**
 * @brief The CAN buses on ACU. Used to select which bus a message is enqueued on.
 */
enum class ACUCANBusses_e
{
    CCU,
    EM
};

/**
 * @brief Owns ACU's CAN buses. This is the only code with access to them: everything else sends by
 * calling enqueueOnSwitch() / enqueueCustomIDOnSwitch() with an already-packed message, and received messages are routed to the
 * interfaces passed in on construction.
 */
class ACUCANInterface : public CANInterface<ACUCANBusses_e>
{
  public:
    ACUCANInterface(CCUInterface& ccu_interface, EMInterface& em_interface, uint32_t ccu_baudrate, uint32_t em_baudrate)
        : _ccu_interface(ccu_interface), _em_interface(em_interface), _ccu_bus(ccu_baudrate), _em_bus(em_baudrate)
    {
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueueOnSwitch(ACUCANBusses_e bus, const CAN_message_t& msg) override;

    /**
     * @brief Drains both RX buffers, routing each message to its interface
     */
    void processAllRxBuffers() override;

    /**
     * @brief Writes every message in both TX buffers out onto its bus
     */
    void sendAllTxBuffers() override;

  private:
    CCUInterface& _ccu_interface;
    EMInterface& _em_interface;

    CANBus<CAN2> _ccu_bus;
    CANBus<CAN3> _em_bus;

    /**
     * @brief Handles a message from the EM bus: forwards it onto the CCU bus, then routes it
     */
    void _onEMBusRecv(const CAN_message_t& msg);

    /**
     * @brief Routes a received message to the appropriate interface based on CANID
     */
    void _recvSwitch(const CAN_message_t& msg);
};

using ACUCANInterfaceInstance = etl::singleton<ACUCANInterface>;

#endif // CORE_CAN_INTERFACE_HPP
