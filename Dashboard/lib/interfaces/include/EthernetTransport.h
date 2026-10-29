#pragma once

#include <cstddef>
#include <cstdint>

struct IPv4Address
{
    uint8_t octexts[4];
};

struct UdpEndpoint
{
    // Supply the actual destination address and port from network configuration.
    IPv4Address address;
    uint16_t port;
};

enum class EthernetStatus
{
    NotInitialized,
    Initializing,
    LinkDown,
    Ready,
    Error
};

enum class IpMode
{
    Static,
    Dhcp
};

struct EthernetConfig
{
    uint8_t phy_address;
    uint8_t mac_address[6];

    IpMode ip_mode;
    IPv4Address local_address;
    IPv4Address subnet_mask;
    IPv4Address gateway;
}

// Platform-neutral UDP transport contract; this header does not implement Ethernet.
// A concrete STM32 backend is pending verified RMII pins, PHY reset/address details,
// and integration with the selected network stack (CubeMX may provide the setup).
class EthernetTransport
{
public:
    virtual ~EthernetTransport() = default;

    // TODO: define whether true means initialized or merely initialization started.
    virtual bool begin() = 0;

    // Service the network stack without blocking the firmware scheduler.
    virtual void poll() = 0;

    virtual EthernetStatus status() const = 0;

    // TODO: define whether success means queued or fully transmitted, and whether
    // the implementation copies data before returning (otherwise data must stay valid).
    virtual bool send_udp(
        const UdpEndpoint& destination,
        const uint8_t* data,
        size_t length) = 0;
};