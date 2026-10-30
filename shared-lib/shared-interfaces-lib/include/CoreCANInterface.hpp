#ifndef CORE_CAN_INTERFACE
#define CORE_CAN_INTERFACE

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
class CoreCANInterface
{
  public:
    virtual ~CoreCANInterface() = default;

    /**
     * @brief Sets up every CAN bus on the board
     */
    virtual void init() = 0;

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
 * @tparam CAN_DEV the FlexCAN peripheral (CAN1, CAN2, CAN3)
 * @tparam RX_BUF_SIZE number of messages the RX buffer holds
 * @tparam TX_BUF_SIZE number of messages the TX buffer holds
 */
template <CAN_DEV_TABLE CAN_DEV, uint16_t RX_BUF_SIZE = 16, uint16_t TX_BUF_SIZE = 128>
class CANBus
{
  public:
    using RecvHandler_t = etl::delegate<void(const CAN_message_t& msg)>;

    CANBus() = default;
    CANBus(const CANBus&) = delete;
    CANBus& operator=(const CANBus&) = delete;

    /**
     * @brief Starts the bus, sets the baudrate, enables the FIFO and its interrupt, and registers this
     * instance with the receive ISR. Call once before using the bus.
     */
    void init(uint32_t baudrate)
    {
        _instance = this;
        _can.begin();
        _can.setBaudRate(baudrate);
        _can.setMaxMB(16);
        _can.enableFIFO();
        _can.enableFIFOInterrupt();
        _can.onReceive(&CANBus::_onRecv);
    }

    /**
     * @brief Enqueues an already-packed CAN message onto the TX buffer
     */
    void enqueue(const CAN_message_t& msg)
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
            _can.write(msg);
        }
    }

  private:
    static constexpr size_t _CAN_MSG_SIZE = sizeof(CAN_message_t);
    static inline CANBus* _instance = nullptr; // one per CAN_DEV instantiation

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

    FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16> _can;
    Circular_Buffer<uint8_t, RX_BUF_SIZE, _CAN_MSG_SIZE> _rx_buffer;
    Circular_Buffer<uint8_t, TX_BUF_SIZE, _CAN_MSG_SIZE> _tx_buffer;
};

#endif /* CORE_CAN_INTERFACE */