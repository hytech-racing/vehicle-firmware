# Drivebrain UART command decoder

The UART interfaces in this directory decode the existing `hytech_msgs.MCUCommandData` directly from the
wire format implemented by Drivebrain's `abdur/uart-driver` branch:

```text
COBS(protobuf payload | CRC16 little-endian) | 0x00
```

CRC-16/CCITT-FALSE covers payload only (polynomial 0x1021, init 0xffff,
no reflection, xorout 0). There is no opening delimiter, type byte, or handshake.

`UARTCommandDecoder::feed(byte, now_millis, output)` returns true only after a
complete, validated command. It checks COBS, CRC, protobuf decoding, both nested
vectors' presence, finite RPMs, and finite nonnegative torque allowances.
Wheel order is FL, FR, RL, RR, with RPM and N*m units. Signed RPMs are preserved;
physical vehicle limits are a future controller responsibility. The legacy
`prev_MCU_recv_millis` is preserved without treating it as a sequence number.

The returned `received_millis` is the caller's **MCU-local processing time**.
Rejected frames never modify output or refresh its timestamp. This is not a
measurement of when the Pi generated a command. A valid but delayed/replayed
command cannot be identified by this schema alone.

The decoder allocates no dynamic memory. Payloads are capped at 128 bytes (the
pinned production command is at most 55 bytes); buffers and parser work are
bounded. Oversize input is discarded through the next zero delimiter. Empty
delimiters are ignored. After malformed/truncated input, framing resumes at a
delimiter; a frame merged onto a truncated frame may also be rejected. `reset()`
discards partial input without clearing counters. There is no partial-frame
timer or idle heartbeat. Counters are uint32_t and wrap; access them on the same
thread as feed/poll.

The decoder exposes commands without accessing global VCR state, CAN, or motor
controllers. It does not implement source arbitration, freshness expiry, fault
latching, or mode switching. These remain required before production actuation.

## Teensy adapter

```cpp
#include "UARTCommandSerial.hpp"
UARTCommandSerial uart(Serial1);
// In setup:
// uart.begin(921600);
// In loop / scheduler:
UARTCommandDecoder::Command command;
if (uart.poll(millis(), command)) {
    // command.message.desired_rpms.FL, command.message.torque_limit_nm.FL, etc.
}
```

`poll` reads at most its byte budget and returns on the first valid command.
Subsequent commands remain in the peripheral buffer until the next poll; drain
often enough for the chosen rate. It does not maintain or overwrite a hidden
latest-command slot. Any HardwareSerial instance can be injected, with supported
pin configuration performed by the caller before begin.

## Bare Teensy receive example

`main_uart_decode_test.cpp` uses Serial1 at 921600, 8N1 and prints aggregate
decoder counts once per second over USB Serial when USB has room. It does not
wait for USB, start VCR systems, or actuate motors. This initial example only
receives; UART replies, dynamic USB configuration, the Pi sender, and the laptop
coordinator/stress suite are deferred for code review.

Wiring (devices separately powered by USB):

| Raspberry Pi 4 | Bare Teensy 4.1 |
|---|---|
| Physical header pin 8, GPIO14/TX | Pin 0, Serial1 RX |
| Physical header pin 10, GPIO15/RX | Pin 1, Serial1 TX |
| Physical header pin 6, GND | GND |

Use 3.3 V logic and short leads. Do not join 3.3 V/5 V power outputs. Configure
the Pi's UART hardware and disable its serial console before traffic.

Build only (does not flash):

```sh
pio run -e vcr_uart_test
```

The new main is excluded from vcr-prod and vcr_adcs_test. The bare-Teensy
build ignores the full interfaces library and explicitly compiles only
`UARTCommandDecoder.cpp` using `scripts/build_uart_decoder_test.py`, so moving
these files alongside the other interfaces does not pull in CAN/Ethernet hardware
or the vehicle application. The production decoder
is not scheduled or selected by existing vehicle firmware.

## Software checks

From the firmware repository root:

```sh
cmake -S VCR/test/uart -B build-uart-decoder
cmake --build build-uart-decoder -j4
ctest --test-dir build-uart-decoder --output-on-failure
```

This downloads SHA256-pinned nanopb 0.4.9.1 and the same HT_proto release used by
platformio.ini. It checks the real nanopb decoder and a mock HardwareSerial
adapter. The command/zero hex fixtures were emitted by Drivebrain's actual
UARTComms and independently checked by its Python COBS/CRC/protobuf reference.
Regenerate with the sibling Drivebrain tests/uart executable's `--fixtures`
option, then convert command.frame and zero.frame to space-separated hex.
Tests cover every wheel value, zero-vector presence, legacy/local timestamps,
invalid COBS/CRC/protobuf, NaN/infinity/negative allowances, missing vectors,
oversize rejection, truncation recovery, reset, and poll budgeting.

These checks do not establish real UART electrical reliability, sustained rate,
or performance under the VCR scheduler. No hardware test is implied.
