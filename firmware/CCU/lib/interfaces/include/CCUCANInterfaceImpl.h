#ifndef CCUCANINTERFACEIMPL
#define CCUCANINTERFACEIMPL

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>

/* Local Interface Includes  */
#include "ACUInterface.h"
#include "ChargerInterface.h"
#include "EMInterface.h"

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
                            ChargerInterface &charger_int,
                            EnergyMeterInterface & em_int
    ) : acu_interface(acu_int),
        charger_interface(charger_int),
        em_interface(em_int)
    {};

    ACUInterface& acu_interface;
    ChargerInterface& charger_interface;
    EnergyMeterInterface& em_interface;

    float max_pack_voltage = 0.0f;
    float cell_cutoff_voltage = 0.0f;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;
struct CCUCANInterface_s
{
    explicit CCUCANInterface_s(etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> recv_switch_func) : can_recv_switch(recv_switch_func) {}

    FlexCAN_t<CAN1> ACU_CAN;
    CANRXBuffer_t acu_can_rx_buffer;
    CANTXBuffer_t acu_can_tx_buffer;

    FlexCAN_t<CAN3> CHARGER_CAN;
    CANRXBuffer_t charger_can_rx_buffer;
    CANTXBuffer_t charger_can_tx_buffer;

    etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> can_recv_switch;
};
using CCUCANInterfaceInstance = etl::singleton<CCUCANInterface_s>;

namespace CCUCANInterfaceImpl
{
    void on_acu_can_receive(const CAN_message_t &msg);

    void on_charger_can_receive(const CAN_message_t &msg);

    /**
     * @brief Routes a decoded message to the appropriate interface based on CANID
     */
    void ccu_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
};

#endif // CCUCANINTERFACEIMPL