#ifndef CAN_INTERFACE
#define CAN_INTERFACE

#include <FlexCAN_T4.h>
#include <etl/delegate.h>
#include <array>
#include <cstdint>
#include <cstring>


/**
 * @brief Base class for each board's CAN interface
 *
 * This only defines the public API. The board owns its CANBus members and decides how received
 * messages are routed, either with one handler per bus or a single shared one.
 */
template <typename CANBus_e>
class CANInterfaceTemplate_t
{
  public:
    virtual ~CANInterfaceTemplate_t() = default;

    /**
     * @brief Enqueues an already-packed CAN message onto the given bus' TX buffer
     */
    virtual void enqueueSwitch(CANBus_e bus, const CAN_message_t& msg) = 0;

    /**
     * @brief Packs a CAN struct and enqueues it onto the given bus' TX buffer via enqueueSwitch()
     * @param msg the unpacked CAN struct (e.g. PEDALS_SYSTEM_DATA_t)
     * @param pack_function the generated pack function (e.g. Pack_PEDALS_SYSTEM_DATA_ht_can)
     */
    template <typename can_struct>
    void packAndEnqueueOnSwitch(CANBus_e bus, can_struct* msg, uint32_t (*pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*))
    {
        CAN_message_t can_msg;
        can_msg.id = pack_function(msg, can_msg.buf, &can_msg.len, (uint8_t*)&can_msg.flags.extended); // NOLINT
        enqueueSwitch(bus, can_msg);
    }

    /**
     * @brief Same as above, but overrides the packed message's ID. Used when one generated struct is shared
     * by several IDs (e.g. all four inverters pack with the INV1 functions)
     * @param id the CAN ID to send the message with, replacing the one the pack function returns
     */
    template <typename can_struct>
    void packAndEnqueueOnSwitch(CANBus_e bus, can_struct* msg, uint32_t (*pack_function)(can_struct*, uint8_t*, uint8_t*, uint8_t*), uint32_t id)
    {
        CAN_message_t can_msg;
        pack_function(msg, can_msg.buf, &can_msg.len, (uint8_t*)&can_msg.flags.extended); // NOLINT
        can_msg.id = id;
        enqueueSwitch(bus, can_msg);
    }

    /**
     * @brief Drains every RX buffer on the board, routing each message to its interface
     */
    virtual void processAllRxBuffers() = 0;

    /**
     * @brief Writes every message in every TX buffer out onto its bus
     */
    virtual void sendAllTxBuffers() = 0;
};

/**
 * @brief Owns a single FlexCAN bus along with its RX and TX buffers.
 *
 * Receiving: ISR pushes each message into the RX buffer, and processRxBuffer() later drains it,
 *            handing each message to a board-specific handler.
 * Sending: enqueue() pushes into the TX buffer, and sendTxBuffer() writes it all out onto the bus.
 * Forwarding: a handler can pass a received message straight to another bus' enqueue().
 *
 * @note
 * - Construction starts the hardware (begin, baudrate, FIFO, receive interrupt), so do NOT construct a
 *   CANBus as a global or as a member of a global
 *
 * - Templated only on CAN_DEV so there is exactly one CANBus type, and one _instance, per peripheral.
 *   FlexCAN has a single receive callback per peripheral, so only one CANBus may exist per CAN_DEV;
 *   constructing a second one halts the board.
 *
 * - Since it is only templated on CAN_DEV, we cannot change RX and TX buffer sizes on construction. This is
 *   acceptable since we have sized it for max, and we do not stress Teensy RAM
 * @tparam CAN_DEV the FlexCAN peripheral (CAN1, CAN2, CAN3)
 */
template <CAN_DEV_TABLE CAN_DEV>
class CANBus
{
  public:
    using RecvHandler_t = etl::delegate<void(const CAN_message_t& msg)>;

    /**
     * @brief Starts the bus, sets the baudrate, enables the FIFO and its interrupt, and registers this
     * instance with the receive ISR. Halts if a CANBus already exists for this CAN_DEV.
     */
    explicit CANBus(uint32_t baudrate)
    {
        if (_instance != nullptr)
        {
            // Programming error: a second CANBus on this peripheral would steal the first's RX callback
            Serial.println("FATAL: CANBus constructed twice for the same CAN_DEV");
            while (true) {}
        }
        _instance = this;
        _bus.begin();
        _bus.setBaudRate(baudrate);
        _bus.setMaxMB(16);
        _bus.enableFIFO();
        _bus.enableFIFOInterrupt();
        _bus.onReceive(&CANBus::_onRecv);
    }

    CANBus(const CANBus&) = delete;
    CANBus& operator=(const CANBus&) = delete;

    /**
     * @brief Enqueues an already-packed CAN message onto the TX buffer
     */
    void enqueueOnBus(const CAN_message_t& msg)
    {
        std::array<uint8_t, _CAN_MSG_SIZE> temp_buffer;
        memmove(temp_buffer.data(), &msg, _CAN_MSG_SIZE);
        _tx_buffer.push_back(temp_buffer.data(), _CAN_MSG_SIZE);
    }

    /**
     * @brief Drains the RX buffer, passing each message to the handler
     * @param handler the board's receive function, expected to switch on the message ID
     */
    void processRxBuffer(RecvHandler_t handler)
    {
        while (_rx_buffer.available())
        {
            CAN_message_t recvd_msg;
            std::array<uint8_t, _CAN_MSG_SIZE> temp_buffer;
            _rx_buffer.pop_front(temp_buffer.data(), _CAN_MSG_SIZE);
            memmove(&recvd_msg, temp_buffer.data(), _CAN_MSG_SIZE);
            handler(recvd_msg);
        }
    }

    /**
     * @brief Writes every message in the TX buffer out onto the bus
     */
    void sendTxBuffer()
    {
        CAN_message_t msg;
        while (_tx_buffer.available())
        {
            std::array<uint8_t, _CAN_MSG_SIZE> temp_buffer;
            _tx_buffer.pop_front(temp_buffer.data(), _CAN_MSG_SIZE);
            memmove(&msg, temp_buffer.data(), _CAN_MSG_SIZE);
            _bus.write(msg);
        }
    }

  private:
    static constexpr size_t _CAN_MSG_SIZE = sizeof(CAN_message_t);
    static constexpr uint16_t _RX_BUF_SIZE = 256; // messages; must be a power of 2
    static constexpr uint16_t _TX_BUF_SIZE = 128; // messages; must be a power of 2
    static inline CANBus* _instance = nullptr; // one per CAN_DEV

    /**
     * @brief FlexCAN receive callback. Static because FlexCAN only accepts plain function pointers
     */
    static void _onRecv(const CAN_message_t& msg)
    {
        if (_instance == nullptr)
        {
            return;
        }
        std::array<uint8_t, _CAN_MSG_SIZE> temp_buffer;
        memmove(temp_buffer.data(), &msg, _CAN_MSG_SIZE);
        _instance->_rx_buffer.push_back(temp_buffer.data(), _CAN_MSG_SIZE);
    }

    FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_128> _bus;
    Circular_Buffer<uint8_t, _RX_BUF_SIZE, _CAN_MSG_SIZE> _rx_buffer;
    Circular_Buffer<uint8_t, _TX_BUF_SIZE, _CAN_MSG_SIZE> _tx_buffer;
};

#endif /* CORE_CAN_INTERFACE */