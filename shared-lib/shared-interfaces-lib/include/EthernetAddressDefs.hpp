#ifndef ETHERNET_ADDRESS_DEFS_HPP
#define ETHERNET_ADDRESS_DEFS_HPP

#include "QNEthernet.h"
#include "etl/singleton.h"

/**
 * Definitions of ethernet IPs according to the BookStack page ("HT09 Ethernet IPs and Ports").
 *
 * USAGE INSTRUCTIONS: In your implementation, create an instance of this
 * struct and use the default definitions for each IP.
 */
struct EthernetIPDefs_s
{
    const IPAddress debug_ip{192, 168, 3, 21};
    const IPAddress drivebrain_ip{192, 168, 3, 30};
    const IPAddress vcr_ip{192, 168, 3, 31};
    const IPAddress vcf_ip{192, 168, 3, 32};
    const IPAddress acu_ip{192, 168, 3, 33};
    const IPAddress ccu_ip{192, 168, 3, 40};
    const IPAddress default_dns{192, 168, 3, 1};
    const IPAddress default_gateway{192, 168, 3, 1};
    const IPAddress car_subnet{255, 255, 255, 0};
    const uint16_t vcf_data_port = 4444;
    const uint16_t vcr_data_port = 9999;
    const uint16_t acu_core_data_port = 7777;
    const uint16_t acu_all_data_port = 7766;
    const uint16_t db_data_port = 8888;
    const uint16_t ccu_data_port = 6666;
};

using EthernetIPDefsInstance = etl::singleton<EthernetIPDefs_s>;

#endif // ETHERNET_ADDRESS_DEFS_HPP