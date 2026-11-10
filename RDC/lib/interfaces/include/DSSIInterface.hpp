#ifndef DSSIINTERFACE_H
#define DSSIINTERFACE_H

#include <SharedFirmwareTypes.h>
#include <etl/delegate.h>
#include <etl/singleton.h>

class DSSIInterface
{
  public:
    DSSIInterface(uint8_t yellow_dssi_pin, uint8_t blue_dssi_pin);

    void setDSSI(unsigned long curr_millis, DriverlessSystemState_e dv_state);

  private:
    uint8_t _yellow_dssi_pin;
    uint8_t _blue_dssi_pin;
    unsigned long lastBlinkTime = 0;
    unsigned int _yellowState;
    unsigned int _blueState;
};

using DSSIInterfaceInstance = etl::singleton<DSSIInterface>;

#endif // DSSIINTERFACE_H
