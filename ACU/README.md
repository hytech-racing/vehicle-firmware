# ACU — Accumulator Control Unit

## Overview

The **Accumulator Control Unit (ACU)** is a low voltage board responsible for managing and monitoring the high voltage accumulator (battery pack). It sits between the Battery Management System (BMS), the Tractive System Board (TSB), and the Vehicle Control Rear (VCR), acting as the safety and monitoring layer for the high voltage system.

**Microcontroller:** Teensy 4.1

### Primary Responsibilities

- Precharge sequence management
- Safety Light Board (SLB) logic
- BMS and IMD fault detection
- BSPD (Brake System Plausibility Device) amplification circuitry
- Cell voltage and temperature monitoring
- Cell balancing
- State of Health (SOH) persistence
- Battery data transmission over CAN and Ethernet

---

## Hardware Connections

### CAN Buses

| Bus | Speed | Devices |
|-----|-------|---------|
| CCU CAN | 1 Mbit/s | CCU |
| EM CAN | 500 kbit/s | Energy Meter |

### SPI — BMS Chips

The ACU communicates with **12 BMS chips** over SPI using two chip select lines.

| Chip Select | Chips |
|-------------|-------|
| CS0 | BMS 0–5 |
| CS1 | BMS 6–11 |

Overall the system monitors:

- 126 battery cells
- 48 temperature sensors

---

## CAN Interface

### CCU CAN (1 Mbit/s)

| Message | Direction | Frequency | Description |
|----------|-----------|-----------|-------------|
| ACU_OK | TX | 20 Hz | Health status |
| BMS_VOLTAGES | TX | 10 Hz | Pack voltage/current/SOC |
| ALL_CELL_VOLTAGES | TX | 10 Hz | 126 cell voltages |
| ALL_CELL_TEMPERATURES | TX | 10 Hz | 48 temperatures |

### EM CAN (500 kbit/s)

| Message | Direction | Frequency | Description |
|----------|-----------|-----------|-------------|
| EM_MEASUREMENT | RX/TX | 250 Hz | Energy meter measurements |

---

## Ethernet Interface

The ACU uses **QNEthernet** for UDP telemetry.

### Core Stream (125 Hz)

- Pack voltage
- Pack current
- State of Charge
- Fault status

### Full Stream (10 Hz)

- All cell voltages
- All temperatures
- SOH data

---

## Battery Monitoring

### Sampling Pipeline

```text
sample_bms_data (400 Hz)
        │
        ▼
Read cell voltages and temperatures
        │
        ▼
evaluate_accumulator (50 Hz)
        │
        ├── Fault detection
        ├── SOC calculation
        └── Pack health evaluation
        │
        ▼
write_cell_balancing_config (10 Hz)
        │
        ▼
Update balancing configuration
```

---

## Data Reporting

```text
enqueue_CCU_core_CAN (10 Hz)
        │
        ├── Pack voltage
        ├── Current
        └── SOC

enqueue_CCU_all_voltages_CAN (10 Hz)
        └── 126 cell voltages

enqueue_CCU_all_temps_CAN (10 Hz)
        └── 48 temperatures

enqueue_ACU_ok_CAN (20 Hz)
        └── ACU status

enqueue_EM_measurement_CAN (250 Hz)
        └── Energy meter data

send_CAN (250 Hz)
        └── Transmit queued CAN messages

send_core_ethernet (125 Hz)
        └── Core telemetry

send_all_ethernet (10 Hz)
        └── Full telemetry
```

Messages are first **queued** by the enqueue tasks before being transmitted by `send_CAN()`. This prevents multiple tasks from attempting to write to the CAN peripheral simultaneously.

---

## Safety Systems

### Hardware Watchdog

| Signal | Pin | Requirement |
|--------|-----|-------------|
| TEENSY_OK | 3 | Held HIGH continuously |
| WD_KICK | 4 | Toggle at 250 Hz |

If `WD_KICK` stops toggling for approximately **4 ms**, the watchdog pulls `BMS_OK` LOW, opening the shutdown circuit and disabling the high-voltage system.

---

### IMD Fault Detection

The Isolation Monitoring Device continuously monitors isolation resistance.

If `IMD_OK` (pin 23) goes LOW, the ACU enters the **FAULT** state.

---

### BMS Fault Detection

The ACU enters the **FAULT** state if:

- Cell voltage > **4.20 V**
- Cell voltage < **3.05 V**
- Cell temperature > **60°C**

---

## Cell Balancing

The balancing algorithm is:

1. Read all cell voltages.
2. Find the minimum cell voltage.
3. Any cell more than **20 mV** above the minimum becomes a balancing candidate.
4. Balancing is only enabled when pack temperature is between **35°C** and **50°C**.
5. Configuration is written to each BMS chip at **10 Hz**.

NOTE: We not actually cell balance due to temperature issues. This season we will be fixing this.



# BMS Driver Group

The `BMSDriverGroup` is the low-level driver responsible for communicating with the LTC6811 battery monitoring ICs over SPI. It continuously measures cell voltages and temperatures, manages configuration (such as cell balancing), validates communication using the LTC6811 Packet Error Code (PEC), and stores processed battery data for the rest of the firmware.

The driver is designed as a **non-blocking state machine** that uses **DMA-based SPI transfers**, allowing battery monitoring to run continuously without blocking the CPU.

---

# Architecture

The driver is split into two primary files.

| File | Purpose |
|------|---------|
| `BMSDriverGroup.h` | Public interface, data structures, configuration, enums, and class declarations. |
| `BMSDriverGroup.tpp` | Implementation of SPI communication, state machine, ADC conversions, PEC generation, data processing, and configuration writes. |

---

# Driver Responsibilities

The driver is responsible for:

- Initializing all LTC6811 devices
- Starting ADC conversions
- Reading cell voltage and GPIO registers
- Converting raw ADC values into engineering units
- Computing battery statistics
- Validating SPI communication using PEC
- Writing configuration registers
- Managing cell balancing configuration

---

# Data Flow

The application should continuously call

```cpp
bms.read_data();
```

Rather than reading every value in one blocking transaction, each call advances the driver's internal state machine.

```text
Application
      │
      ▼
 read_data()
      │
      ▼
 State Machine
      │
      ├── Start ADC Conversion
      ├── Wait for Conversion
      ├── Read Register Group
      ├── Process Data
      └── Repeat
```

---

# Driver State Machine

The driver cycles through two independent state machines.

## SPI State

The SPI state controls the communication process.

```text
START_CONVERSIONS
        │
        ▼
WAIT_CONVERSION
        │
        ▼
IDLE
        │
        ▼
WAIT_READ_COMPLETE
        │
        ▼
START_CONVERSIONS
```

Each state performs a small amount of work before returning control back to the application.

This allows the firmware to continue executing while ADC conversions and SPI transfers complete.

---

## Read Groups

The LTC6811 stores measurements in six register groups.

```text
CV Group A   → Cells 1-3
CV Group B   → Cells 4-6
CV Group C   → Cells 7-9
CV Group D   → Cells 10-12
GPIO Group A → GPIO 1-3
GPIO Group B → GPIO 4-5
```

The driver cycles through these continuously.

```text
CV A
 ↓
CV B
 ↓
CV C
 ↓
CV D
 ↓
GPIO A
 ↓
GPIO B
 ↓
repeat
```

A complete measurement cycle consists of reading all six groups.

---

# Initialization

During `init()` the driver performs the following:

1. Configure all chip select pins.
2. Initialize battery data structures.
3. Reset min/max tracking values.
4. Register the DMA callback.
5. Wake all LTC6811 devices.

Once initialization is complete the driver is ready to begin measurements.

---



# Communicating with the LTC6811 / Reading Battery Data

This section is based on the BMDDriverGroup files. The read process consists of several stages...

## 1. Start ADC Conversion

When beginning a new cycle, the driver sends one of the LTC6811 ADC conversion commands.

- Cell Voltage Conversion (`ADCV`)
- GPIO Conversion (`ADAX`)

The LTC6811 then begins sampling internally.

---

## 2. Wait for Conversion

The driver waits for the ADC conversion to finish before attempting to read any registers. This wait is handled using just a simple elapsed timer (elapsed_time > time_expected) rather than blocking delays.

---

## 3. Read Register Group

Once conversion is complete, the driver reads the current register group over SPI using DMA.

Each transfer returns:

- Measurement data
- Packet Error Code (PEC)

---

## 4. Validate Communication

Every received packet includes a 15-bit Packet Error Code (PEC). The driver computes its own PEC and compares it against the received value.

```text
Received Data
      │
      ▼
Calculate PEC
      │
      ▼
Compare
      │
      ├── Valid → Process Data
      └── Invalid → Ignore Packet
```

This protects against corrupted SPI communication caused by EMI or other various transmission errors.

---

## 5. Convert Measurements

The raw ADC values are converted into usable units.

---

## 6. Update Battery Statistics

As measurements are processed, the driver continuously updates:

- Total pack voltage
- Average cell voltage
- Minimum cell voltage
- Maximum cell voltage
- Minimum cell temperature
- Maximum cell temperature
- Maximum board temperature

---

# Writing Configuration

The driver also supports writing configuration registers to every LTC6811.

Configuration includes:

- Under-voltage threshold
- Over-voltage threshold
- ADC operating mode
- GPIO enable bits
- Cell balancing enable bits

Configuration writes are also performed asynchronously through the SPI state machine.

---

# Complete Measurement Cycle

A full battery update follows this sequence.

```text
call read_data()
      │
      ▼
Start ADC Conversion
      │
      ▼
Wait for Conversion
      │
      ▼
Read CV Group A
      │
      ▼
Read CV Group B
      │
      ▼
Read CV Group C
      │
      ▼
Read CV Group D
      │
      ▼
Read GPIO Group A
      │
      ▼
Read GPIO Group B
      │
      ▼
Update Battery Statistics
      │
      ▼
Repeat
```

---