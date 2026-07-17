# VCF — Vehicle Control Front

## Overview

The **Vehicle Control Front (VCF)** is a low voltage board responsible for acquiring the digital and analog signals from the front of the car — pedals, steering, front suspension — and streaming that data out to the rest of the car over CAN and Ethernet. It also drives the dashboard: buttons, buzzer, and status neopixels.

NOTE: Since VCF handles pedal data, it sits in the critical torque path. If the firmware ever freezes or hangs, the watchdog forces the car into shutdown rather than continuing on stale sensor data.

**Microcontroller:** Teensy 4.1

##### Primary Responsibilities

- Reading and validating accelerator/brake pedal signals
- BSPD (Brake System Plausibility Device) monitoring
- Reading redundant analog + digital steering sensors, cross-checked against each other
- Reading front suspension load cells and potentiometers
- Persisting sensor calibration (min/max ranges) to EEPROM
- Driving dashboard buttons, buzzer, and status neopixels
- Streaming processed sensor data over CAN and Ethernet



## Sensors
Overall the system monitors:

- 2 redundant steering sensors (1 analog + 1 digital)
- 4 pedal position sensors (2 accel, 2 brake)
- 2 front suspension load cells
- 2 front suspension potentiometers
- 2 brake pressure sensors (front/rear)
- 2 shutdown circuit voltage taps
- 7 dashboard buttons, 1 buzzer, 16 status neopixels



## Hardware Connections

### CAN Buses

| Bus | Peripheral | Speed |
|-----|------------|-------|
| TELEM CAN | CAN1 | 1 Mbit/s |
| FRONT_AUX_CAN | CAN2 | 500 kbit/s |

### SPI — ADCs

The VCF reads all analog sensors through **two MCP3208 8-channel ADCs**.

| Chip Select | ADC | Channels |
|-------------|-----|----------|
| CS10 | ADC0 | Pedal reference, analog steering sensor (cw/ccw), accel pedal sensor (x2), brake pedal sensor (x2)
| CS38 | ADC1 | Shutdown circuit (×2), load cells (×2), suspension pots (×2), brake pressure sensor (×2)

### I2C — IO Expander

Address 0x20, used for dashboard IO.

---

## ADC Interface

Owns 2 MCP3208 ADC chips and converts raw counts into calibrated usable units.

### Responsibilities

- Own both MCP3208 ADCs (`_adc0`, `_adc1`) and sample them on request
- Apply per-channel scale/offset calibration to every reading
- Run an IIR low-pass filter on load cells and suspension pots
- Provide a clean getter per signal (e.g. `get_acceleration_1()`, `get_FL_load_cell()`) so upstream systems never touch raw ADC counts directly

### Data Flow

```text
tick_adc0()                         tick_adc1()
   │                                    │
   ▼                                    ▼
Sample 7 channels                  Sample 8 channels
(pedal ref, steering cw/ccw,       (shutdown ×2, load cells ×2,
 accel ×2, brake ×2)                suspension pots ×2, brake pressure ×2)
   │                                    │
   └───────────────┬────────────────────┘
                    ▼
      Apply per-channel scale + offset
                    │
                    ▼
      update_filtered_values(alpha)
      → IIR filter on load cells + suspension pots only
                    │
                    ▼
      get_*() accessors return calibrated
      (and, where applicable, filtered) values
```

---

## CAN Interface

VCF's CAN layer is split into receive and transmit paths, each built around circular buffers so the CAN peripheral is never touched directly by application/logic code.

### Receiving

Incoming messages are decoded and routed to the interface that owns that data, rather than being handled inline:

```text
CAN peripheral (ISR)
        │
        ▼
on_telem_can_recv() / on_front_aux_can_recv()
        │
        ▼
vcf_recv_switch(interfaces, msg, millis, interface_type)
        │
        ├── ACUInterface            (battery/accumulator data)
        ├── BrakeRotorTempInterface (brake rotor temperature)
        ├── DashboardInterface      (dashboard inputs)
        └── VCRInterface            (Vehicle Control Rear data)
```

`vcf_recv_switch` looks at the CAN ID and dispatches the decoded message to the matching interface object, which then holds the latest value for the rest of the firmware to read.

### Transmitting

Outbound messages from VCF's own systems (pedals, steering, suspension, dash) are pushed into a per-bus TX circular buffer as they're produced, and `send_all_CAN_msgs()` drains that buffer out to the FlexCAN peripheral.

---

## Ethernet Interface

VCF uses **QNEthernet** over UDP, sending a single structured **protobuf** message.

### Outbound: VCF → rest of car

```text
make_vcf_data_msg()
        │
        ├── ADCInterface            (raw + filtered analog readings)
        ├── DashboardInterface      (dashboard state)
        ├── PedalsSystem            (validated pedal data)
        ├── SteeringSystem          (validated steering angle)
        └── BrakeRotorTempInterface (brake rotor temperature)
        │
        ▼
hytech_msgs_VCFData_s  (protobuf struct)
        │
        ▼
handle_send_ethernet_vcf_data()  →  UDP send socket
```

Rather than each system enqueuing its own Ethernet message independently (as they do for CAN), the whole VCF state is assembled into **one combined protobuf message** at send time and shipped out together — sent via `ethernet_send_task` at 10 Hz.

### Inbound: VCR → VCF

VCF also receives data over Ethernet from the Vehicle Control Rear (VCR) board:

```text
UDP recv socket
        │
        ▼
receive_pb_msg_vcr(msg_in, shared_state, curr_millis)
        │
        ▼
Update VCF shared state
```

NOTE: This inbound path is currently used **only for buzzer control** — VCR tells VCF when to sound the buzzer.

---

## Sampling Pipeline

```text
adc0_sample (2 kHz)
        │
        ▼
ADCInterface::tick_adc0()
Read pedal reference, steering (cw/ccw), accelerator ×2, brake ×2
        │
        ▼
adc1_sample (4 kHz)
        │
        ▼
ADCInterface::tick_adc1()
Read shutdown circuit, load cells, suspension pots, brake pressure
        │
        ▼
Calibrate & scale (raw counts → real units, per-channel scale/offset)
        │
        ▼
Validate
        │
        ├── Cross-check redundant sensors (steering, pedals)
        ├── Reject implausible sample-to-sample jumps
        └── Apply deadzones / activation thresholds
        │
        ▼
Update system state (pedals / steering / suspension)
```

`ADCInterface::update_filtered_values(alpha)` runs an IIR low-pass filter on **both the load cells and the suspension potentiometers** (not just load cells) between calibration and validation, smoothing out noise before the value is trusted.

---

## Data Reporting

```text
pedals_message_enqueue (250 Hz)
        └── Accelerator + brake position

steering_message_enqueue (250 Hz)
        └── Steering angle (both sensors)

front_suspension_message_enqueue (250 Hz)
        └── Load cells + suspension pot travel

dash_CAN_enqueue (10 Hz)
        └── Dashboard button/status state

CAN_send (500 Hz)
        └── Transmit all queued CAN messages

ethernet_send_task (10 Hz)
        └── Stream telemetry over UDP
```

---

## Safety Systems

### Hardware Watchdog

If the kick task stops running on schedule, `SOFTWARE_OK` is no longer asserted and the external watchdog forces the car into shutdown.

### Steering Plausibility

The two steering sensors (analog + digital) are checked against each other every sample — see the **Steering System** explanation below.

### Pedal Plausibility

Accelerator and brake are each read from two redundant sensors and cross-checked, per FSAE EV rules — see the **Pedals System** explanation below.

---

## Calibration

Accelerator, brake, and both steering sensors each have a learned min/max range stored in EEPROM rather than hardcoded, refreshed periodically by dedicated recalibration tasks:

```text
pedals_calibration_task (10 Hz)
        └── Refresh accel/brake min-max EEPROM ranges

steering_calibration_task (10 Hz)
        └── Refresh analog/digital steering min-max EEPROM ranges (with margin)
```

This is likely triggered by the dashboard's recalibration button, and lets each board be calibrated in place rather than requiring identical, pre-matched sensors.

---

## Pedals System

`PedalsSystem` owns evaluation for both the accelerator and brake pedals. Each pedal has **two redundant sensors**, and the system is responsible for scaling, validating, and combining them into a single trusted percentage. NOTE: Timing and tolerances are driven directly by FSAE EV rules.

### Responsibilities

- Scale raw ADC readings into a 0.0–1.0 pedal percentage, with deadzone removed
- Detect an unplugged/disconnected sensor (out-of-range check)
- Detect disagreement between a pedal's two redundant sensors
- Enforce the FSAE 100 ms implausibility shutdown timer
- Detect simultaneous accelerator + brake application (APPS/brake plausibility)
- Track observed sensor range continuously, and support in-place recalibration

### Data Flow

```text
set_pedals_sensor_data()
        │
        ▼
update_observed_pedal_limits()   ← runs continuously, tracks raw min/max per sensor
        │
        ▼
evaluate_pedals(pedal_data, curr_millis)
        │
        ├── Scale + remove deadzone (per sensor)
        ├── Out-of-range check          → unplugged/faulted sensor
        ├── Min/max implausibility check → calibrated range ± implausibility margin
        ├── Sensor agreement check       → accel_1 vs accel_2, brake_1 vs brake_2
        ├── 100 ms implausibility timer  → FSAE Rule T.4.3.3
        └── Accel + brake pressed check  → FSAE Rule T.4.2.5 (APPS/BSPD)
        │
        ▼
PedalsSystemData_s
```

### FSAE Rule Compliance

| Constant | Value | Rule | Purpose |
|----------|-------|------|---------|
| `IMPLAUSIBILITY_PERCENT` | 10% | T.4.2.5 | Max allowed deviation between redundant sensors and from calibrated range before it's flagged implausible |
| `IMPLAUSIBILITY_DURATION` | 100 ms | T.4.3.3 | An implausibility must last for this long before it's treated as a real fault |
| `ACCELERATION_PERCENT_LIMIT` | 5% | — | Accelerator threshold above which a simultaneous brake press triggers an APPS/brake plausibility fault |

### Calibration & Polarity Handling

`update_observed_pedal_limits()` runs on every sample and continuously tracks the min/max raw value seen by each of the four sensors.

When the driver holds the recalibration button (**only valid with both pedals at rest**), `recalibrate_min_max()` is called, which:

1. Compares the current (at-rest) reading against the running observed min and max for each sensor.
2. Whichever the current reading is *closer to* — min or max — tells the system whether that sensor is wired with a **negative-coefficient** (output decreases as the pedal is pressed).
3. Assigns `min_pedal_*` / `max_pedal_*` accordingly, flipping them for negative-coefficient sensors.

---

## Steering System

`SteeringSystem` takes in angles from an analog and a digital steering sensor, then it outputs a single trusted angle, filters noise and rejects implausible readings from either sensor.

### Responsibilities

- Convert raw analog/digital readings into degrees
- Low-pass filter the (noisier) analog signal
- Reject out-of-range readings on either sensor independently
- Reject implausible angle jumps between samples
- Cross-check the two sensors against each other
- Track observed sensor range continuously, and support recalibration of the digital sensor

### Data Flow

```text
update_observed_steering_limits()   ← runs continuously, tracks min/max of analog/digital (raw)
        │
        ▼
evaluate_steering(analog_raw, digital_data, current_millis)
        │
        ├── Convert analog_raw   → degrees
        ├── Low-pass filter analog angle (2nd-order Butterworth, fc = 8 Hz @ fs = 500 Hz)
        ├── Convert digital_raw  → degrees
        ├── Out-of-range check   → analog and digital (done independently)
        ├── dθ check             → reject large improbable changes in angle
        └── Cross-check          → ensure analog and digital are within some tolerance
        │
        ▼
SteeringSystemData_s
```

### Calibration

`update_observed_steering_limits()` continuously tracks raw min/max like the pedals system does. `recalibrate_steering_digital()` provides a way to refresh the digital sensor's calibrated range by holding a dashboard button.

---
