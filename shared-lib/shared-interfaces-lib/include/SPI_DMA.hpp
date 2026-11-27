#ifndef SPI_DMA_HPP
#define SPI_DMA_HPP

/**
 * SPIDMAInterface - generic, non-blocking SPI transfer using the Teensy's DMA.
 * Nothing in it is specific to one chip; device drivers should inherit
 * from it and fill in what to send and how to read the reply.
 *
 * How it works:
 *  - The class owns a fixed-size TX buffer and RX buffer (bufferSize bytes each)
 *    and an EventResponder passed in by reference.
 *  - initDMA() sets the CS pin to OUTPUT and attaches _DMACallback() to the
 *    EventResponder.
 *  - transfer() begins the SPI transaction, pulls CS low, clears the RX buffer and
 *    calls the async spiBus.transfer(tx, rx, n, eventResponder). It sets the busy flag
 *    and returns immediately; the DMA moves the bytes in the background.
 *  - When the DMA finishes, the EventResponder calls _DMACallback(), which calls the
 *    object's transferComplete(): CS goes high, the transaction ends and the busy
 *    flag clears.
 *
 * NOTE: the callback is attached with attachImmediate(), so transferComplete()
 * runs in INTERRUPT context! CPU pauses immediately.
 */

#include <SPI.h>
#include <array>
#include <EventResponder.h>


struct SPIParams_s {
    const uint8_t spiPinCS;
    const uint8_t spiPinSDI;
    const uint8_t spiPinSDO;
    const uint8_t spiPinCLK;
    const int spiSpeed;
    const uint8_t bitOrder = MSBFIRST;  // MSBFIRST or LSBFIRST
    const uint8_t dataMode = SPI_MODE0; // SPI_MODE0..SPI_MODE3, check the chip's datasheet
    SPIClass& spiBus = SPI;             // which SPI controller: SPI or SPI1
};

/**
 * Returns the busy flag for an SPI bus. Every device on the same bus shares one flag, so
 * only one transfer runs on that bus's wires at a time. SPI and SPI1 have separate flags
 * because they are separate hardware and can transfer at the same time.
 *
 * NOTE: Kept outside the template so, for example, SPIDMAInterface<3> and SPIDMAInterface<6>
 *       on the same bus still share the flag.
 */
inline volatile bool& spiBusBusyFlag(SPIClass& bus)
{
    static volatile bool spiBusy = false;
    static volatile bool spi1Busy = false;
    return (&bus == &SPI1) ? spi1Busy : spiBusy;
}

template <size_t bufferSize>
class SPIDMAInterface {
    public:
        SPIDMAInterface(SPIParams_s spiParams, EventResponder& eventRef) :
            _spiParams(spiParams),
            _txBuffer{},
            _rxBuffer{},
            _isDMABusy(false),
            _eventResponder(eventRef)
        {};

        void initDMA();

        /**
         * Starts a DMA transfer of _txBuffer. Returns false (and does nothing) if another
         * device on the same bus is mid-transfer.
         */
        virtual bool transfer();
        virtual void transferComplete();

        void setTxBuffer(const std::array<uint8_t, bufferSize>& txBuffer);

        /**
         * @return True while THIS device's transfer is running.
         */
        bool isDMABusy() const;

        /**
         * @return True while any device on this device's bus is mid-transfer.
         */
        bool isBusBusy() const { return spiBusBusyFlag(_spiParams.spiBus); }

        const std::array<uint8_t, bufferSize>& getRxBuffer() const { return _rxBuffer; }

    protected:
        SPIParams_s _spiParams;
        std::array<uint8_t, bufferSize> _txBuffer;
        std::array<uint8_t, bufferSize> _rxBuffer;
        volatile bool _isDMABusy;
        EventResponder& _eventResponder;

    private:
        static void _DMACallback(EventResponderRef ref);

};

#include "SPI_DMA.tpp"

#endif // SPI_DMA_HPP