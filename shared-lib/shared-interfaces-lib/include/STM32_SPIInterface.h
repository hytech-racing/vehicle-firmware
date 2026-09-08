#ifndef __STM32_SPI_INTERFACE_H__
#define __STM32_SPI_INTERFACE_H__

/**
 * @brief Generic, per-board-configurable SPI interface for STM32 (HAL-based).
 *
 *  Design goals:
 *    - One reusable interface; each board passes its own config (which SPI
 *      peripheral, pins, bus params, transfer mode, DMA streams).
 *    - Supports Blocking / Interrupt / DMA transfer modes.
 *    - Supports multiple devices (chip-selects) sharing one bus.
*/

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>


enum class SPIMode_e : uint8_t
{
    Blocking,
    Interrupt,
    DMA
};

/**
 * @brief Everything a board specifies to configure one SPI bus
 * @note Fields under "DMA only" are ignored unless mode == DMA
 * @note Add CRC fields if needed,
*/
struct SPIConfig_s
{
    // Which peripheral + Transfer mode
    SPI_TypeDef* instance = nullptr;   // SPI1, SPI2, SPI3, ...
    SPIMode_e mode = SPIMode_e::Blocking;

    // Bus parameters (map straight onto HAL SPI_InitTypeDef)
    uint32_t baud_rate_prescaler = SPI_BAUDRATEPRESCALER_16;
    uint32_t clock_polarity = SPI_POLARITY_LOW;
    uint32_t clock_phase = SPI_PHASE_1EDGE;
    uint32_t data_size = SPI_DATASIZE_8BIT;
    uint32_t first_bit = SPI_FIRSTBIT_MSB;

    // Pins used by the interface's MspInit
    uint32_t sck_pin = NC;
    uint32_t miso_pin = NC;
    uint32_t mosi_pin = NC;

    // DMA only (ignored unless mode == DMA)
    DMA_Stream_TypeDef* dma_tx_stream = nullptr;   // e.g. DMA1_Stream0
    DMA_Stream_TypeDef* dma_rx_stream = nullptr;
    uint32_t dma_tx_request = 0;  // DMA_REQUEST_SPIx_TX
    uint32_t dma_rx_request = 0;  // DMA_REQUEST_SPIx_RX
};


/**
 * @brief One device on the bus. Each has its own chip-select pin
*/
struct SPIDevice_s
{
    uint32_t cs_pin = NC;
    bool is_cs_active_low = true;
};


class SPIInterface
{
public:

    SPIInterface() = default;

    /**
     * @brief Configure + initialize the bus
     * @return True if configured, false if there is no SPI instance or the HAL is not ok
    */
    bool init(const SPIConfig_s& config);

    /** Full-duplex transfer to a device. Asserts CS, transfers, deasserts CS.
     *  Blocking mode: returns when done.
     *  Interrupt/DMA mode: returns immediately; poll isTxComplete().
     *  Pass rxBuf = nullptr for transmit-only, txBuf = nullptr for receive-only. */
    bool transfer(const SPIDevice_s& device, uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length);

    void mspInit();  // configures pins + clocks for this instance


    /// @return True once the last non-blocking transfer has finished, false otherwise
    bool isTxTransferComplete() const { return _is_tx_transfer_complete; }

    SPI_HandleTypeDef* getSPIHandle() { return &_hspi; }

    // --- called by the global callbacks/IRQ shims; not for user code ---
    void _onTxComplete() { _is_tx_transfer_complete = true; _deassertCS(); }


private:

    void _assertCS();
    void _deassertCS();
    SPIConfig_s _config;
    SPI_HandleTypeDef _hspi   = {};
    DMA_HandleTypeDef _hdma_tx = {};
    DMA_HandleTypeDef _hdma_rx = {};
    volatile bool _is_tx_transfer_complete = true;
    const SPIDevice_s* _active_device = nullptr;
};

namespace HT_SPI_Registry
{
    bool add(SPI_TypeDef* instance, SPIInterface* obj);
    SPIInterface* find(SPI_TypeDef* instance);
}

#endif /* HT_SPI_H */