#include "STM32_SPIInterface.h"

/**
 * @note The main problem is that HAL callbacks are global, but our SPI objects are not.
 *       When a SPI transfer finishes, HAL calls the same function regardless of the SPI bus.
 * @note We just need a way for. the HAL to know our objects exist and what SPI bus correlates to that object.
 * @note This is only useful when we are running more than one SPI bus, which is possible
*/
namespace
{
    struct RegisterEntry_s
    {
        SPI_TypeDef* instance;
        SPIInterface* spi_object;
    };

    ///@note Bump this if a board ever uses more than 3 SPI buses at once.
    constexpr size_t max_spi_buses = 3;
    RegisterEntry_s g_registry[max_spi_buses] = {};
}

namespace HT_SPI_Registry
{

    /**
     * @brief Methods adds a SPI bus to the registry
     * @param instance is the actual hardware bus, so SPI1, SPI2, etc.
     * @param spi_object is the object encapsulating the instance
     * @return True if bus successfully added or replaced/updated, false otherwise
     * @note If return false, need to increase max_spi_buses for the registry
    */
    bool add(SPI_TypeDef* instance, SPIInterface* spi_object)
    {
        // Replace if already present, else take a free slot.
        for (auto& entry : g_registry)
        {
            if (entry.instance == instance)
            {
                entry.spi_object = spi_object;
                return true;
            }
        }
        for (auto& entry : g_registry)
        {
            if (entry.instance == nullptr)
            {
                entry.instance = instance;
                entry.spi_object = spi_object;
                return true;
            }
        }

        return false;  // table full — increase max_spi_buses
    }

    /**
     * @brief Methods finds the spi_object encapsulting a specific hardware bus
     * @return A pointer to the spi_object
    */
    SPIInterface* find(SPI_TypeDef* instance)
    {
        for (auto& entry : g_registry)
        {
            if (entry.instance == instance)
            {
                return entry.spi_object;
            }
        }

        return nullptr;
    }

} // namespace HT_SPI_Registry



bool SPIInterface::init(const SPIConfig_s& config)
{
    _config = config;

    if (_config.instance == nullptr)
    {
        return false;
    }


    /**
     * Register this SPI object and its corresponding bus BEFORE HAL_SPI_Init() because HAL_SPI_Init() triggers
     * HAL_SPI_MspInit(), which looks into our registry
    */
    if (!HT_SPI_Registry::add(_config.instance, this))
    {
        return false;
    }

    _hspi.Instance               = _config.instance;
    _hspi.Init.Mode              = SPI_MODE_MASTER;
    _hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    _hspi.Init.DataSize          = _config.data_size;
    _hspi.Init.CLKPolarity       = _config.clock_polarity;
    _hspi.Init.CLKPhase          = _config.clock_phase;
    _hspi.Init.NSS               = SPI_NSS_SOFT;   // we drive CS in software per-device
    _hspi.Init.BaudRatePrescaler = _config.baud_rate_prescaler;
    _hspi.Init.FirstBit          = _config.first_bit;
    _hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    _hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    _hspi.Init.CRCPolynomial     = 0x0;

    if (HAL_SPI_Init(&_hspi) != HAL_OK)
    {
        return false;
    }

    // No transfer occured, however, want to signal that the bus is free
    _is_tx_transfer_complete = true;
    return true;
}

void SPIInterface::_assertCS()
{
    if (_active_device && _active_device->cs_pin != NC)
    {
        digitalWrite(_active_device->cs_pin, _active_device->is_cs_active_low ? LOW : HIGH);
    }
}

void SPIInterface::_deassertCS()
{
    if (_active_device && _active_device->cs_pin != NC)
    {
        digitalWrite(_active_device->cs_pin, _active_device->is_cs_active_low ? HIGH : LOW);
    }
}

bool SPIInterface::transfer(const SPIDevice_s& device, uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length)
{
    if (device.cs_pin != NC)
    {
        pinMode(device.cs_pin, OUTPUT);
        digitalWrite(device.cs_pin, device.is_cs_active_low ? HIGH : LOW); // set idle
    }

    _active_device  = &device;
    _is_tx_transfer_complete = false;
    _assertCS();

    HAL_StatusTypeDef curr_status = HAL_ERROR;

    switch (_config.mode)
    {
        case SPIMode_e::Blocking:
        {
            if (tx_buffer && rx_buffer)
            {
                curr_status = HAL_SPI_TransmitReceive(&_hspi, tx_buffer, rx_buffer, length, HAL_MAX_DELAY);
            }
            else if (tx_buffer)
            {
                curr_status = HAL_SPI_Transmit(&_hspi, tx_buffer, length, HAL_MAX_DELAY);
            }
            else if (rx_buffer)
            {
                curr_status = HAL_SPI_Receive(&_hspi, rx_buffer, length, HAL_MAX_DELAY);
            }
            _deassertCS();  // blocking: transfer done, drop CS now
            _is_tx_transfer_complete = true;
            break;
        }
        case SPIMode_e::Interrupt:
        {
            if (tx_buffer && rx_buffer)
            {
                curr_status = HAL_SPI_TransmitReceive_IT(&_hspi, tx_buffer, rx_buffer, length);
            }
            else if (tx_buffer)
            {
                curr_status = HAL_SPI_Transmit_IT(&_hspi, tx_buffer, length);
            }
            else if (rx_buffer)
            {
                curr_status = HAL_SPI_Transmit_IT(&_hspi, rx_buffer, length);
            }
            // CS dropped in _onTxComplete() when the IRQ fires.
            break;
        }
        case SPIMode_e::DMA:
        {
            if (tx_buffer && rx_buffer)
            {
                curr_status = HAL_SPI_TransmitReceive_DMA(&_hspi, tx_buffer, rx_buffer, length);
            }
            else if (tx_buffer)
            {
                curr_status = HAL_SPI_Transmit_DMA(&_hspi, tx_buffer, length);
            }
            else if (rx_buffer)
            {
                curr_status = HAL_SPI_Transmit_DMA(&_hspi, rx_buffer, length);
            }
            // CS dropped in _onTxComplete() when the DMA-complete IRQ fires.
            break;
        }
    }

    if (curr_status != HAL_OK)
    {
        _deassertCS();
        _is_tx_transfer_complete = true;
        return false;
    }

    return true;
}

void SPIInterface::mspInit()
{
    // Peripheral clock enable, instance-specific.
    if (_config.instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();
    }
    else if (_config.instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
    }
    else if (_config.instance == SPI3)
    {
        __HAL_RCC_SPI3_CLK_ENABLE();
    }

    // Pin AF config: let STM32duino wire the pins to this SPI instance.
    // pinmap_pinout() applies the correct alternate function from the core's
    // PinMap tables, so we don't hand-code AF numbers per board.
    if (_config.sck_pin  != NC)
    {
        pinmap_pinout(digitalPinToPinName(_config.sck_pin),  PinMap_SPI_SCLK);
    }
    if (_config.miso_pin != NC)
    {
        pinmap_pinout(digitalPinToPinName(_config.miso_pin), PinMap_SPI_MISO);
    }
    if (_config.mosi_pin != NC)
    {
        pinmap_pinout(digitalPinToPinName(_config.mosi_pin), PinMap_SPI_MOSI);
    }

    // DMA setup (only if configured for DMA).
    if (_config.mode == SPIMode_e::DMA)
    {
        // NOTE: DMA controller clock must be enabled. On H7 most SPI DMA is on
        // DMA1/DMA2 — enable both defensively; harmless if already on.
        __HAL_RCC_DMA1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        if (_config.dma_tx_stream)
        {
            _hdma_tx.Instance                 = _config.dma_tx_stream;
            _hdma_tx.Init.Request             = _config.dma_tx_request;
            _hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
            _hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
            _hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
            _hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            _hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
            _hdma_tx.Init.Mode                = DMA_NORMAL;
            _hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
            _hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
            HAL_DMA_Init(&_hdma_tx);
            __HAL_LINKDMA(&_hspi, hdmatx, _hdma_tx);
        }
        if (_cfg.dmaRxStream) {
            _hdmaRx.Instance                 = _cfg.dmaRxStream;
            _hdmaRx.Init.Request             = _cfg.dmaRxRequest;
            _hdmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
            _hdmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
            _hdmaRx.Init.MemInc              = DMA_MINC_ENABLE;
            _hdmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            _hdmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
            _hdmaRx.Init.Mode                = DMA_NORMAL;
            _hdmaRx.Init.Priority            = DMA_PRIORITY_LOW;
            _hdmaRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
            HAL_DMA_Init(&_hdmaRx);
            __HAL_LINKDMA(&_hspi, hdmarx, _hdmaRx);
        }
    }
}

extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    SPIInterface* spi_object = HT_SPI_Registry::find(hspi->Instance);
    if (spi_object)
    {
        spi_object->mspInit();
    }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPIInterface* spi_object = HT_SPI_Registry::find(hspi->Instance);
    if (spi_object)
    {
        spi_object->_onTxComplete();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPIInterface* spi_object = HT_SPI_Registry::find(hspi->Instance);
    if (spi_object)
    {
        spi_object->_onTxComplete();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPIInterface* spi_object = HT_SPI_Registry::find(hspi->Instance);
    if (spi_object)
    {
        spi_object->_onTxComplete();
    }
}