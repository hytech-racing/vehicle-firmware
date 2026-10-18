#include "SPI_DMA.h"

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::dmaCallback(SPIClass* SPI) {
    digitalWrite(_spiParams.spiPinCS, HIGH);
    SPI->endTransaction();
    _dmaBusy = false;
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::initDMA() {
    pinMode(_spiPinCS, OUTPUT);
    _eventRef->setContext(this);
    _eventRef->attachImmediate(transferComplete);
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::transfer(SPIClass* SPI) {
    SPI->beginTransaction(SPISettings(_spiParams.spiSpeed, MSBFIRST, SPI_MODE0));
    digitalWrite(_spiParams.spiPinCS, LOW);
    _rxBuf.fill(0);
    SPI->transfer(_txBuf.data(), _rxBuf.data(), bufferSize, _eventRef);
    _dmaBusy = true;

}