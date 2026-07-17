ACU — Accumulator Control Unit
Overview
The Accumulator Control Unit (ACU) is a low voltage board responsible for managing and monitoring the high voltage accumulator (battery pack). It sits between the Battery Management System (BMS), the Tractive System Board (TSB), and the Vehicle Control Rear (VCR), acting as the safety and monitoring layer for the high voltage system.
Microcontroller: Teensy 4.1 (IMXRT1062, 600MHz ARM Cortex-M7)
Primary responsibilities:

Precharge sequence management
Safety Light Bar (SLB) logic
BMS and IMD fault detection
BSPD (Brake System Plausibility Device) circuitry
Cell voltage and temperature monitoring via BMS chips
Cell balancing
State of Health (SOH) persistence
Battery data transmission over CAN and ethernet