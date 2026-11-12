#include "EthernetAddressDefs.hpp"
#include "EthernetTestHarness.hpp"

/**
 * ACU Ethernet test (192.168.3.33). Connect to another board running its ethernet test and watch serial.
 */
void setup()
{
    EthernetIPDefsInstance::create();
    EthernetTestHarness::setup(EthernetIPDefsInstance::instance().acu_ip);
}

void loop()
{
    EthernetTestHarness::loop();
}
