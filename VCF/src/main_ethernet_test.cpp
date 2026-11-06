#include "EthernetAddressDefs.h"
#include "EthernetTestHarness.h"

/**
 * VCF Ethernet test (192.168.3.32). Connect to another board running its ethernet test and watch serial.
 */
void setup()
{
    EthernetIPDefsInstance::create();
    EthernetTestHarness::setup(EthernetIPDefsInstance::instance().vcf_ip);
}

void loop()
{
    EthernetTestHarness::loop();
}
