#ifndef VCFCANINTERFACE_H
#define VCFCANINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "CANInterface.hpp"
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "BrakeRotorTempInterface.hpp"
#include "DashboardInterface.hpp"
#include "VCRInterface.hpp"


/**
 * @brief The CAN buses on VCF. Used to select which bus a message is enqueued on.
 */
enum class VCFCANBusses_e
{
    TELEM,
    FRONT_AUX
};

/**
 * @brief Owns VCF's CAN buses. This is the only code with access to them: everything else sends by
 * calling enqueue(), and received messages are routed to the interfaces passed in on construction.
 */
class VCFCANInterface : public CANInterface<VCFCANBusses_e>
{
  public:
    VCFCANInterface(uint32_t telem_baudrate, uint32_t front_aux_baudrate)
        : _telem_bus(telem_baudrate), _front_aux_bus(front_aux_baudrate)
    {
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    void enqueueOnSwitch(VCFCANBusses_e bus, const CAN_message_t& msg) override;

    /**
     * @brief Drains both RX buffers, routing each message to its interface
     */
    void processAllRxBuffers() override;

    /**
     * @brief Writes every message in both TX buffers out onto its bus
     */
    void sendAllTxBuffers() override;

  private:
    CANBus<CAN1> _telem_bus;
    CANBus<CAN2> _front_aux_bus;

    /**
     * @brief Routes a received message to the appropriate interface based on CANID. Shared by both buses.
     */
    void _recvSwitch(const CAN_message_t& msg);
};

using VCFCANInterfaceInstance = etl::singleton<VCFCANInterface>;

#endif // VCFCANINTERFACE_H
