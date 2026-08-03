#ifndef ETHERNET_ADDRESS_DEFS_H
#define ETHERNET_ADDRESS_DEFS_H

#include "QNEthernet.h"
#include "etl/singleton.h"

/**
 * Definitions of ethernet IPs according to the BookStack page ("HT09 Ethernet IPs and Ports").
 * 
 * USAGE INSTRUCTIONS: In your implementation, create an instance of this
 * struct and use the default definitions for each IP.
 */
struct EthernetIPDefs_s {
    const IPAddress debug_ip{192, 168, 1, 21};
    const IPAddress drivebrain_ip{192, 168, 1, 30};
    const IPAddress vcr_ip{192, 168, 1, 31};
    const IPAddress vcf_ip{192, 168, 1, 32};
    const IPAddress acu_ip{192, 168, 1, 33};
    const IPAddress ccu_ip{192, 168, 1, 40};
    const IPAddress default_dns{192, 168, 1, 1};
    const IPAddress default_gateway{192, 168, 1, 1};
    const IPAddress car_subnet{255, 255, 255, 0};
    const uint16_t VCFData_port = 4444;
    const uint16_t VCRData_port = 9999;
    const uint16_t ACUCoreData_port = 7777;
    const uint16_t ACUAllData_port = 7766;
    const uint16_t DBData_port = 8888;
};

using EthernetIPDefsInstance = etl::singleton<EthernetIPDefs_s>;

#endif /* ETHERNET_ADDRESS_DEFS_H */