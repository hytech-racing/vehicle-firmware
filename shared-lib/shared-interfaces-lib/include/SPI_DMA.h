#ifndef SPIDMA_H_
#define SPIDMA_H_

#include <SPI.h>
#include <array>
#include <EventResponder.h>


struct SPIParams_s {
    const uint8_t spiPinCS;
    const uint8_t spiPinSDI;
    const uint8_t spiPinSDO;
    const uint8_t spiPinCLK;
    const int spiSpeed;
};

template <size_t bufferSize>
class SPIDMAInterface {
    public:
        SPIDMAInterface(SPIParams_s spiParams, EventResponder& eventRef, std::array<uint8_t, bufferSize> txBuf, std::array<uint8_t, bufferSize> rxBuf) : 
            _spiParams(spiParams), 
            _txBuf(txBuf),
            _rxBuf(rxBuf),
            _dmaBusy(false),
            _eventRef(eventRef)
        {};

        void initDMA();

        virtual void transfer();

        virtual void dmaCallback();

        bool getDMAState() { return _dmaBusy; }

        void setTxBuf(std::array<uint8_t, bufferSize>& txBuf);

        const std::array<uint8_t, bufferSize>& getRxBuf() { return _rxBuf; }

    protected:
        SPIParams_s _spiParams;
        std::array<uint8_t, bufferSize> _txBuf;
        std::array<uint8_t, bufferSize> _rxBuf;
        bool _dmaBusy;
        EventResponder _eventRef;

};

#include "SPI_DMA.tpp"

#endif // SPIDMA_H_