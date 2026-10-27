#ifndef SPIDMA_H_
#define SPIDMA_H_

#include <SPI.h>
#include <array>
#include <EventResponder.h>


namespace spi_params_defaults
{
    constexpr const uint8_t ADC_DEFAULT_SPI_CS    = 10;
    constexpr const uint8_t ADC_DEFAULT_SPI_SDI   = 12;
    constexpr const uint8_t ADC_DEFAULT_SPI_SDO   = 11;
    constexpr const uint8_t ADC_DEFAULT_SPI_CLK   = 13;
    constexpr const int ADC_DEFAULT_SPI_SPEED = 2000000;
};

struct SPIParams_s {
    const uint8_t spiPinCS  = spi_params_defaults::ADC_DEFAULT_SPI_CS;
    const uint8_t spiPinSDI = spi_params_defaults::ADC_DEFAULT_SPI_SDI;
    const uint8_t spiPinSDO = spi_params_defaults::ADC_DEFAULT_SPI_SDO;
    const uint8_t spiPinCLK = spi_params_defaults::ADC_DEFAULT_SPI_CLK;
    const int spiSpeed      = spi_params_defaults::ADC_DEFAULT_SPI_SPEED;
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
        virtual void transferComplete();


        bool getDMAState() { return _dmaBusy; }

        void setTxBuf(std::array<uint8_t, bufferSize>& txBuf);

        const std::array<uint8_t, bufferSize>& getRxBuf() { return _rxBuf; }

    protected:
        SPIParams_s _spiParams;
        std::array<uint8_t, bufferSize> _txBuf;
        std::array<uint8_t, bufferSize> _rxBuf;
        volatile bool _dmaBusy;
        EventResponder& _eventRef;
    private:
        static void dmaCallback(EventResponderRef ref);

};

#include "SPI_DMA.tpp"

#endif // SPIDMA_H_