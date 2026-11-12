#ifndef STM32_CAN_H
#define STM32_CAN_H

/**
 * @file STM32_CANInterface.h
 * @brief Generic, per-board-configurable classic CAN interface for STM32H7 (FDCAN peripheral, HAL-based).
 *
 *  Design goals:
 *    - One reusable driver; each board passes its own config (which FDCAN peripheral, pins, kernel
 *      clock, bit timing) and registers its own receive callback.
 *    - Classic CAN 2.0 only (8-byte frames, no bit-rate switching), standard 11-bit IDs.
 *    - Supports FDCAN1 and FDCAN2 at the same time.
 *
 * @note Teensy boards use CANInterface.h (FlexCAN_T4) instead; the two must not be included together,
 *       since both define CAN_message_t.
 *
 * @note The driver owns the global HAL hooks (FDCANx_IT0_IRQHandler, HAL_FDCAN_RxFifo0Callback), so a
 *       board must not define its own. It is only compiled when the board sets -D HT_USE_SHARED_STM32_CAN
 *       in platformio.ini, so boards with their own FDCAN driver (e.g. Dashboard's HT_FDCAN) keep working
 *       until they move over.
*/

#if defined(ARDUINO_ARCH_STM32)

//For older HAL Driver, utilized different names, aliased for cross compatibility.
#define FDCAN_TypeDef FDCAN_GlobalTypeDef

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include <Arduino.h>
#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include <stm32h7xx_hal_fdcan.h>
#include <etl/singleton.h>


typedef struct
{
    uint32_t id;
    uint8_t extended;
    uint8_t dlc;
    uint8_t data[64];
    uint8_t* buf;
    uint8_t len; //defines similar fields twice like len and dlc for compatibility between STM and Teensy
} CAN_message_t;


/**
 * @brief Everything a board specifies to configure one CAN bus
 * @note Bit rate = kernel clock / nominal_prescaler / (1 + nominal_time_seg1 + nominal_time_seg2).
 *       e.g. PDB: 80 MHz (PLL1Q) / 10 / (1 + 13 + 2) = 500 kbit/s
 *            Dashboard: 25 MHz (HSE) / 1 / (1 + 42 + 7) = 500 kbit/s
 * @note FDCAN1 and FDCAN2 share one message RAM. When using both, give FDCAN2 a message_ram_offset
 *       past FDCAN1's area (FDCAN1 uses 1 filter + RX FIFO0 + TX FIFO, sized by the element counts below).
*/
struct STM32CANConfig_s
{
    // Which peripheral
    FDCAN_GlobalTypeDef* instance = nullptr;    // FDCAN1, FDCAN2

    // Pins (Arduino pin names, e.g. PD0 / PD1); alternate function comes from the core's PinMap_CAN tables
    uint32_t rx_pin = NC;
    uint32_t tx_pin = NC;

    // Kernel clock feeding the peripheral; the bit timing below is computed from it
    uint32_t kernel_clock_source = RCC_FDCANCLKSOURCE_HSE;  // RCC_FDCANCLKSOURCE_HSE / _PLL (PLL1Q) / _PLL2 (PLL2Q)

    // Nominal bit timing (map straight onto HAL FDCAN_InitTypeDef)
    uint16_t nominal_prescaler = 1;
    uint8_t nominal_sync_jump_width = 1;
    uint8_t nominal_time_seg1 = 1;
    uint8_t nominal_time_seg2 = 1;

    bool auto_retransmission = true;            // CAN default: resend frames that weren't acknowledged

    // FIFO sizes (frames)
    uint8_t rx_fifo_elements = 8;
    uint8_t tx_fifo_elements = 8;

    uint32_t message_ram_offset = 0;            // Only needed when FDCAN1 and FDCAN2 are both used
    uint32_t irq_priority = 0;                  // NVIC priority of the receive interrupt (0 = highest)
};


/**
 * @brief Called once per received frame
 * @warning Runs in interrupt context: keep it short (copy the frame, route it, set flags)
*/
using STM32CANReceiveCallback = void (*)(const CAN_message_t &msg);


class STM32CANInterface
{
public:

    STM32CANInterface() = default;

    STM32CANInterface(const STM32CANInterface &)            = delete;
    STM32CANInterface &operator=(const STM32CANInterface &) = delete;

    /**
     * @brief Configures clocks, pins, bit timing and an accept-all filter, then starts the bus
     *        with the receive interrupt enabled.
     * @note Call setReceiveCallback() first if frames should be handled from the start.
     * @return True if the bus started, false on a bad config or a HAL error
    */
    bool init(const STM32CANConfig_s &config);

    /**
     * @brief Registers the function called for every received frame. nullptr = drop received frames.
    */
    void setReceiveCallback(STM32CANReceiveCallback callback) { _receive_callback = callback; }

    /**
     * @brief Queues a classic CAN frame with a standard 11-bit ID
     * @param len 0-8 bytes
     * @return True if queued, false if len > 8, data is null, or the TX FIFO is full
    */
    bool write(uint32_t id, const uint8_t *data, uint8_t len);

    FDCAN_HandleTypeDef *getHandle() { return &_hfdcan; }

    // --- called by the global HAL hooks; not for user code ---
    void _onRxFifo0();

private:

    STM32CANConfig_s _config;
    FDCAN_HandleTypeDef _hfdcan = {};
    STM32CANReceiveCallback _receive_callback = nullptr;
    
};

using STM32CANInterfaceInstance = etl::singleton<STM32CANInterface>;

#endif // ARDUINO_ARCH_STM32

#endif // STM32_CAN_H
