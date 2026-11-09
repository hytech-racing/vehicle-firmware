"""
Laptop side of the board Ethernet test, for when you only have ONE board on the bench.

The normal Ethernet test is board-to-board: flash two boards with their <board>_ethernet_test env,
connect them, and watch each board's serial monitor (see shared-lib/shared-interfaces-lib/include/
EthernetTestHarness.h). This script lets a laptop stand in for the second board by sending it one
8-byte UDP packet; the board prints it on its serial monitor.

Your laptop must be on the car network: a static IP of 192.168.3.x (not one a board uses, e.g.
192.168.3.50) with subnet mask 255.255.255.0, plugged into the board or the car switch.

Usage:
    python3 scripts/test_ethernet.py <board_ip> 5555          # 5555 = EthernetTestHarness::TEST_PORT
    e.g. python3 scripts/test_ethernet.py 192.168.3.32 5555   # VCF
"""

import socket
import argparse
import os

def send_udp_packet(ip: str, port: int):
    data = bytes([0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69])

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(data, (ip, port))
        print(f"Sent {data.hex()} to {ip}:{port}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a UDP packet with 8 bytes.")
    parser.add_argument("ip", type=str, help="Destination IP address")
    parser.add_argument("port", type=int, help="Destination port number")

    args = parser.parse_args()
    send_udp_packet(args.ip, args.port)