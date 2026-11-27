#include "SPI_DMA.hpp"


template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::initDMA() {
    pinMode(_spiParams.spiPinCS, OUTPUT);
    _eventResponder.setContext(this);
    _eventResponder.attachImmediate(&_DMACallback);
}

template <size_t bufferSize>
bool SPIDMAInterface<bufferSize>::transfer() {
    volatile bool& busBusy = spiBusBusyFlag(_spiParams.spiBus);
    if (busBusy) {
        return false; // another device on this bus is using the wires
    }
    // Claim the bus BEFORE starting the DMA so the completion interrupt can never
    // clear the flags before they are set.
    busBusy = true;
    _isDMABusy = true;

    _spiParams.spiBus.beginTransaction(SPISettings(_spiParams.spiSpeed, _spiParams.bitOrder, _spiParams.dataMode));
    digitalWrite(_spiParams.spiPinCS, LOW);
    _rxBuffer.fill(0);
    _spiParams.spiBus.transfer(_txBuffer.data(), _rxBuffer.data(), bufferSize, _eventResponder);
    return true;
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::transferComplete() {
    digitalWrite(_spiParams.spiPinCS, HIGH);
    _spiParams.spiBus.endTransaction();
    _isDMABusy = false;
    spiBusBusyFlag(_spiParams.spiBus) = false; // release the bus last
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::setTxBuffer(const std::array<uint8_t, bufferSize>& txBuffer) {
    _txBuffer = txBuffer;
}

template <size_t bufferSize>
bool SPIDMAInterface<bufferSize>::isDMABusy() const {
    return _isDMABusy;
}

template <size_t bufferSize>
void SPIDMAInterface<bufferSize>::_DMACallback(EventResponderRef ref) {
    auto* self = static_cast<SPIDMAInterface<bufferSize>*>(ref.getContext());
    self->transferComplete();
}