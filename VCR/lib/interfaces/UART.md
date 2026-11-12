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

## Bare Teensy UART bench

`vcr_uart_test` builds the endpoint in `VCR/test/uart_bench/firmware`. The laptop
coordinator configures its UART port, baud rate, and mode over USB. In raw mode,
the endpoint decodes and echoes valid `MCUCommandData` commands; in bench mode,
it handles the test request and reply envelopes. It reports counters over USB
and does not start VCR systems or actuate motors. The default suite uses Serial1
at 921600 baud. See the sibling Drivebrain repository's
`tests/hil/uart/README.md` for setup, wiring, and running the suite.

Wiring (devices separately powered by USB):

| Raspberry Pi 4 | Bare Teensy 4.1 |
|---|---|
| Physical header pin 8, GPIO14/TX | Pin 0, Serial1 RX |
| Physical header pin 10, GPIO15/RX | Pin 1, Serial1 TX |
| Physical header pin 6, GND | GND |

Use 3.3 V logic and short leads. Do not join 3.3 V/5 V power outputs. Configure
the Pi's UART hardware and disable its serial console before traffic.

The build requires generated nanopb messages and a schema identity in
`.pio/uart_bench_generated`. The coordinator's `setup` command generates these
files and builds the firmware; `--no-flash` also checks an already installed
endpoint and requires the Pi and Teensy to be connected. To generate the files
and build locally without connecting or flashing hardware, run the following
from the firmware repository root, with sibling `drivebrain` and `HT_proto`
checkouts and `protoc` available:

```sh
uv run --with nanopb==0.4.9.1 --with 'protobuf>=5.29,<8' \
  ../drivebrain/tests/hil/uart/generate_messages.py \
  --ht-proto ../HT_proto/proto --out .pio/uart_bench_generated --nanopb
pio run -e vcr_uart_test
```

The bare-Teensy build ignores the full interfaces library and uses
`scripts/build_uart_decoder_test.py` to compile the decoder, generated messages,
and bench firmware without the vehicle application. The production decoder is
not scheduled or selected by existing vehicle firmware.

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
