#include "EthernetAddressDefs.h"
#include "EthernetTestHarness.h"

/**
 * CCU Ethernet test (192.168.3.40). Connect to another board running its ethernet test and watch serial.
 */
void setup()
{
    EthernetIPDefsInstance::create();
    EthernetTestHarness::setup(EthernetIPDefsInstance::instance().ccu_ip);
}

void loop()
{
    EthernetTestHarness::loop();
}
