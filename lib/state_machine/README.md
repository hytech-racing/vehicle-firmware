# Vehicle State Machine

## Overview
The **Vehicle State Machine** switches the car between its high-level states. It is responsible for commanding the lower-level drivetrain state machine, activating the buzzer when necessary, and handling other state-dependent operations.

## Expected Usage
The Vehicle State Machine is designed to be independent of other systems. For this reason, when constructing it, the user is required to pass in `etl::delegate` instances (callback mechanisms) that provide methods for handling state outputs or transitions.

### Example: Creating a Delegate
Below is the correct syntax for creating an `etl::delegate`, which would eventually be passed into the drivetrain system:

```c++
etl::delegate<bool()> brake_pressed = etl::delegate<bool()>::create([]() -> bool {
    return pedal_system->get_brake_pressed();
});

VehicleStateMachine vehicle_state_machine(..., brake_pressed, ...);
```

## State Machine Diagram
Copy the following into a Mermaid-compatible viewer to visualize the Vehicle State Machine:

```
---
title: Vehicle State Machine
---
stateDiagram-v2

    ---
title: Vehicle State Machine
---
stateDiagram-v2

    tsna : TRACTIVE_SYSTEM_NOT_ACTIVE
    tsa : TRACTIVE_SYSTEM_ACTIVE
    wrtd : WANTING_READY_TO_DRIVE
    rtd : READY_TO_DRIVE
    wcp : WANTING_CALIBRATE_PEDALS
    cp : CALIBRATING_PEDALS
    wcs : WANTING_CALIBRATE_STEERING
    cs : CALIBRATING_STEERING

    tsna --> tsa : If HV is over threshold
    tsna --> wcp : Pedal calibration button is pressed
    tsna --> wcs : Steering calibration button is pressed

    wcs --> cs : Steering calibration button held for > 3000ms
    wcs --> tsna : Steering calibration button released within 3000ms

    wcp --> cp : Pedal calibration button held for >3000ms
    wcp --> tsna : Pedal calibration button released within 3000ms

    cs --> tsna : Steering Calibration button released

    cp --> tsna : Pedal calibration button released

    tsa --> tsna : If HV is under threshold
    tsa --> wrtd : If the brake and start button are pressed

    wrtd --> tsna : If HV is under threshold
    wrtd --> rtd : If the drivetrain system is in the ready state

    rtd --> tsna : If HV is under threshold
    rtd --> tsa : If a drivetrain error occurs
```
