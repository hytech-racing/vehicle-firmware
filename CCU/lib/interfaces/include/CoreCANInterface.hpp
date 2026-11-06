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
#include "ChargerInterface.hpp"
#include "EMInterface.hpp"


/**
 * @brief The CAN buses on CCU. Used to select which bus a message is enqueued on.
 */
enum class CCUCANBusses_e
{
    ACU,
    CHARGER
};

/**
 * @brief Owns CCU's CAN buses. This is the only code with access to them: everything else sends by
 * calling enqueueSwitch() / packAndEnqueueOnSwitch(), and received messages are routed to the
 * interfaces passed in on construction.
 */
class CCUCANInterface : public CANInterface<CCUCANBusses_e>
{
  public:
    CCUCANInterface(
        uint32_t acu_baudrate,
        uint32_t charger_baudrate)
        : _acu_bus(acu_baudrate),
          _charger_bus(charger_baudrate)
    {
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueueOnSwitch(CCUCANBusses_e bus, const CAN_message_t& msg) override;

    /**
     * @brief Drains both RX buffers, routing each message to its interface
     */
    void processAllRxBuffers() override;

    /**
     * @brief Writes every message in both TX buffers out onto its bus
     */
    void sendAllTxBuffers() override;

  private:
    // TODO: never set anywhere, so these stay at 0.0. Carried over as-is from the old CANInterfaces_s
    float _max_pack_voltage = 0.0f;
    float _cell_cutoff_voltage = 0.0f;

    CANBus<CAN1> _acu_bus;
    CANBus<CAN3> _charger_bus;

    /**
     * @brief Routes a received message to the appropriate interface based on CANID. Shared by both buses.
     */
    void _recvSwitch(const CAN_message_t& msg);
};

using CCUCANInterfaceInstance = etl::singleton<CCUCANInterface>;

#endif // CORE_CAN_INTERFACE_HPP