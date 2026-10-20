#ifndef SPIDMA_H_
#define SPIDMA_H_

#include <SPI.h>

using namespace std;


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
        SPIDMAInterface(SPIParams_s spiParams, EventResponder* eventRef, array<uint8_t, bufferSize> txBuf, array<uint8_t, bufferSize> rxBuf) : 
            _spiParams(spiParams), 
            _eventRef(eventRef),
            _dmaBusy(false),
            _txBuf(txBuf),
            _rxBuf(rxBuf) 
        {};

        void initDMA();

        virtual void transfer(SPIClass* SPI);

        virtual void dmaCallback(SPIClass* SPI);

        bool getDMAState() { return _dmaBusy; }

        void setTxBuf(array<uint8_t, bufferSize> txBuf);

        array<uint8_t, bufferSize> getRxBuf() { return _rxBuf; }

    protected:
        SPIParams_s _spiParams;
        array<uint8_t, bufferSize> _txBuf;
        array<uint8_t, bufferSize> _rxBuf;
        bool _dmaBusy;
        EventResponder* _eventRef;

};


#endif // SPIDMA_H_