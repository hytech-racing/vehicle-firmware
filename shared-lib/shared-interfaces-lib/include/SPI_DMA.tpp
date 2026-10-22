#include "SPI_DMA.h"

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::transferComplete() {
    digitalWrite(_spiParams.spiPinCS, HIGH);
    SPI.endTransaction();
    _dmaBusy = false;
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::dmaCallback(EventResponderRef ref) {
    auto* self = static_cast<SPIDMAInterface<bufferSize>*>(ref.getContext());
    self->transferComplete();     
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::initDMA() {
    pinMode(_spiParams.spiPinCS, OUTPUT);
    _eventRef.setContext(this);
    _eventRef.attachImmediate(&dmaCallback);
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::transfer() {
    SPI.beginTransaction(SPISettings(_spiParams.spiSpeed, MSBFIRST, SPI_MODE0));
    digitalWrite(_spiParams.spiPinCS, LOW);
    _rxBuf.fill(0);
    SPI.transfer(_txBuf.data(), _rxBuf.data(), bufferSize, _eventRef);
    _dmaBusy = true;
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::setTxBuf(std::array<uint8_t, bufferSize>& txBuf) {
    _txBuf = txBuf;
}