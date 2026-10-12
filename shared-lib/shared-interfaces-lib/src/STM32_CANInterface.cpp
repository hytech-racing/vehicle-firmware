#include "STM32_CANInterface.hpp"

// Opt-in: this file owns global HAL hooks, so only boards that set -D HT_USE_SHARED_STM32_CAN compile it
#if defined(ARDUINO_ARCH_STM32) && defined(HT_USE_SHARED_STM32_CAN)

/**
 * @note HAL interrupt handlers and callbacks are global, but our CAN objects are not. The registry maps
 *       each FDCAN peripheral to the object driving it, so the global hooks can find the right one.
*/
namespace
{
    struct RegisterEntry_s
    {
        FDCAN_GlobalTypeDef* instance;
        STM32CANInterface* can_object;
    };

    constexpr size_t max_can_buses = 2;  // FDCAN1, FDCAN2
    RegisterEntry_s g_registry[max_can_buses] = {};

    /**
     * @return True if added or replaced, false if the registry is full
    */
    bool registryAdd(FDCAN_GlobalTypeDef* instance, STM32CANInterface* can_object)
    {
        for (auto& entry : g_registry)
        {
            if (entry.instance == instance)
            {
                entry.can_object = can_object;
                return true;
            }
        }
        for (auto& entry : g_registry)
        {
            if (entry.instance == nullptr)
            {
                entry.instance = instance;
                entry.can_object = can_object;
                return true;
            }
        }
        return false;
    }

    STM32CANInterface* registryFind(FDCAN_GlobalTypeDef* instance)
    {
        for (auto& entry : g_registry)
        {
            if (entry.instance == instance)
            {
                return entry.can_object;
            }
        }
        return nullptr;
    }

    // Classic CAN: the HAL encodes 0-8 bytes as FDCAN_DLC_BYTES_0..8
    constexpr uint32_t DLC_FROM_LENGTH[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
}


bool STM32CANInterface::init(const STM32CANConfig_s &config)
{
    _config = config;

    if (_config.instance != FDCAN1 && _config.instance != FDCAN2)
    {
        return false;
    }

    // Register before starting, so the first receive interrupt can find this object
    if (!registryAdd(_config.instance, this))
    {
        return false;
    }

    // Kernel clock (shared by FDCAN1 and FDCAN2) + peripheral clock
    RCC_PeriphCLKInitTypeDef clock_init = {};
    clock_init.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clock_init.FdcanClockSelection = _config.kernel_clock_source;
    if (HAL_RCCEx_PeriphCLKConfig(&clock_init) != HAL_OK)
    {
        return false;
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();

    // Pins: the core's PinMap_CAN tables supply the alternate function, so no AF numbers per board
    if (_config.rx_pin == NC || _config.tx_pin == NC)
    {
        return false;
    }
    pinmap_pinout(digitalPinToPinName(_config.rx_pin), PinMap_CAN_RD);
    pinmap_pinout(digitalPinToPinName(_config.tx_pin), PinMap_CAN_TD);

    _hfdcan.Instance = _config.instance;
    _hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    _hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    _hfdcan.Init.AutoRetransmission = _config.auto_retransmission ? ENABLE : DISABLE;
    _hfdcan.Init.TransmitPause = DISABLE;
    _hfdcan.Init.ProtocolException = DISABLE;
    _hfdcan.Init.NominalPrescaler = _config.nominal_prescaler;
    _hfdcan.Init.NominalSyncJumpWidth = _config.nominal_sync_jump_width;
    _hfdcan.Init.NominalTimeSeg1 = _config.nominal_time_seg1;
    _hfdcan.Init.NominalTimeSeg2 = _config.nominal_time_seg2;
    _hfdcan.Init.DataPrescaler = 1;            // Data phase is only used by CAN FD frames
    _hfdcan.Init.DataSyncJumpWidth = 1;
    _hfdcan.Init.DataTimeSeg1 = 1;
    _hfdcan.Init.DataTimeSeg2 = 1;
    _hfdcan.Init.MessageRAMOffset = _config.message_ram_offset;
    _hfdcan.Init.StdFiltersNbr = 1;            // One accept-all filter, configured below
    _hfdcan.Init.ExtFiltersNbr = 0;
    _hfdcan.Init.RxFifo0ElmtsNbr = _config.rx_fifo_elements;
    _hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxFifo1ElmtsNbr = 0;
    _hfdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxBuffersNbr = 0;
    _hfdcan.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.TxEventsNbr = 0;
    _hfdcan.Init.TxBuffersNbr = 0;
    _hfdcan.Init.TxFifoQueueElmtsNbr = _config.tx_fifo_elements;
    _hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    _hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&_hfdcan) != HAL_OK)
    {
        return false;
    }

    // Accept every standard ID into RX FIFO0: (ID & mask 0) == (filter 0 & mask 0) is always true
    FDCAN_FilterTypeDef filter = {};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&_hfdcan, &filter) != HAL_OK)
    {
        return false;
    }

    // Receive interrupt: new frame in FIFO0 -> FDCANx_IT0_IRQHandler -> HAL_FDCAN_RxFifo0Callback
    const IRQn_Type irq = (_config.instance == FDCAN1) ? FDCAN1_IT0_IRQn : FDCAN2_IT0_IRQn;
    HAL_NVIC_SetPriority(irq, _config.irq_priority, 0);
    HAL_NVIC_EnableIRQ(irq);

    if (HAL_FDCAN_ActivateNotification(&_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        return false;
    }

    return HAL_FDCAN_Start(&_hfdcan) == HAL_OK;
}

bool STM32CANInterface::write(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (_hfdcan.Instance == nullptr || len > 8 || (data == nullptr && len > 0))
    {
        return false;
    }

    FDCAN_TxHeaderTypeDef tx_header = {};
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;               // 11-bit ID
    tx_header.TxFrameType = FDCAN_DATA_FRAME;           // Data, not a remote request
    tx_header.DataLength = DLC_FROM_LENGTH[len];
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // Only sent in CAN FD frames; no effect here
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan, &tx_header, const_cast<uint8_t *>(data)) == HAL_OK;
}

void STM32CANInterface::_onRxFifo0()
{
    FDCAN_RxHeaderTypeDef rx_header;

    // Drain the FIFO: several frames can arrive before the interrupt is serviced
    while (HAL_FDCAN_GetRxFifoFillLevel(&_hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        CAN_message_t msg = {};
        if (HAL_FDCAN_GetRxMessage(&_hfdcan, FDCAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK)
        {
            break;  // Don't spin on a faulty frame
        }
        msg.id = rx_header.Identifier;
        msg.extended = (rx_header.IdType == FDCAN_EXTENDED_ID);
        msg.dlc = static_cast<uint8_t>(rx_header.DataLength);
        msg.len = msg.dlc;                                  // Classic CAN: DLC 0-8 is the byte count
        msg.buf = msg.data;

        if (_receive_callback != nullptr)
        {
            _receive_callback(msg);
        }
    }
}


// ------------------------------- Global HAL hooks -------------------------------

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    {
        return;
    }
    STM32CANInterface* can_object = registryFind(hfdcan->Instance);
    if (can_object != nullptr)
    {
        can_object->_onRxFifo0();
    }
}

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
    STM32CANInterface* can_object = registryFind(FDCAN1);
    if (can_object != nullptr)
    {
        HAL_FDCAN_IRQHandler(can_object->getHandle());
    }
}

extern "C" void FDCAN2_IT0_IRQHandler(void)
{
    STM32CANInterface* can_object = registryFind(FDCAN2);
    if (can_object != nullptr)
    {
        HAL_FDCAN_IRQHandler(can_object->getHandle());
    }
}

#endif // ARDUINO_ARCH_STM32 && HT_USE_SHARED_STM32_CAN
