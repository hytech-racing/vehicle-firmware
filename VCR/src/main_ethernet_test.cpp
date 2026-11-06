#include "EthernetAddressDefs.h"
#include "EthernetTestHarness.h"

/**
 * VCR Ethernet test (192.168.3.31). Connect to another board running its ethernet test and watch serial.
 */
void setup()
{
    EthernetIPDefsInstance::create();
    EthernetTestHarness::setup(EthernetIPDefsInstance::instance().vcr_ip);
}

void loop()
{
    EthernetTestHarness::loop();
}
