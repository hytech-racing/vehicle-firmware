
## Drivetrain System (tentative design docs)

### List of features

- [X] Ability to initialize inverters
    - User must call `evaluate_drivetrain()` and pass in an instance of `DrivetrainInit_s`
- [X] Ability to command inverters
    - User may call `evaluate_drivetrain()` and pass in a `DrivetrainCommand_s`
- [ ] Ability to switch inverter control modes*
    - All commands are technically speed commands. See the `README.md` in the interfaces library.
- [ ] Ability to detect timeout of initialization of inverters
- [ ] Ability to change parameters of inverters
- [X] Detailed error status for invalid usage of drivetrain system
    - User may call `get_status().inverter_statuses`
- [X] Ability to get all data available from inverter
    - User may call `get_status().inverter_statuses`

### Usage Description

The user must call the `evaluate_drivetrain` function. This will:
1) Update the state machine (internal to drivetrain)
2) Update the command in each InverterInterface
3) Return the current state of the Drivetrain state machine (DSM)

#### Initialization
The user must call the `evaluate_drivetrain` function with the `CmdVariant` type set to `DrivetrainInit_s`. The user should continuously call the `evaluate_drivetrain` with this struct until the drivetrain's state reaches the expected state.

#### Drivetrain Commanding

Once initialized, the user may command the drivetrain with either speed or torque commands*.
- All commands are technically speed commands. See the `README.md` in interfaces library.

### State Machine

Instead of the drivetrain system being a direct API interface on top of the inverter interface with calls to the drivetrain system that call the lower-level inverter interface immediately to queue CAN messages, the drivetrain system will instead update pieces of the inverter interface's internal state (such as bit flags) for it to send periodically.

This allows more control over the send rate of the InverterInterface (necessary to manage CAN saturation).


```
---
title: Drivetrain State Machine
---
stateDiagram-v2

    nc : NOT_CONNECTED 
    not_en : NOT_ENABLED_NO_HV_PRESENT
    not_en_hv : NOT_ENABLED_HV_PRESENT
    ready : INVERTERS_READY 
    hv_en : INVERTERS_HV_ENABLED
    en : INVERTERS_ENABLED
    drive_mode : ENABLED_DRIVE_MODE
    err: ERROR
    clear_err: CLEARING_ERRORS
    note2: note that any state other than NOT_CONNECTED can go to the ERROR state if in any of the states any inverter has an error flag present
    
    note right of err
        during this state all setpoints will be set to 0 and the inverter control word will have the inverter_enable flag set to false
    end note

    note right of clear_err
        during this state all setpoints will be set to 0, (inverter_enable: false, hv_enable: true, driver_enable: true, remove_error: true)
    end note

    note right of drive_mode
        during this mode the drivetrain is free to receive speed commands and car will move
    end note

    
    nc --> not_en: received data from all inverters
    nc --> not_en_hv: received data from all inverters AND the inverters have a voltage above HV level
    
    not_en --> not_en_hv: HV present to all inverters
    not_en_hv --> ready: all inverters have their ready flag set

    hv_en --> ready: on quit dc flag off

    ready --> hv_en: requesting initialization of drivetrain AND all inverters have their quit dc flag on
    ready --> not_en_hv: if any of the inverters have their ready flags not set
    hv_en --> en: on set of all inverters enabled flags
    en --> hv_en: on inverter enabled flag off

    en --> drive_mode: on command request of drive mode
    drive_mode --> err: on error during operation
    drive_mode --> err: incorrect user command type OR any inverter error
    err --> clear_err: on user request of error reset 
    clear_err --> not_en_hv: on successful reset of errors (either internally to the drivetrain system or of the inverters themselves)
```

## Drivebrain Controller System

Drivebrain Controller for fail-safe pass-through control of the car
  
This class is the "controller" that allows pass-through control as commanded by the DriveBrain. It also calculates the latency of the most recent input and
checks if a command has "expired". If the input has expired, it switches over to the fail-safe control mode (MODE0) to allow for safe failing even while the car is driving.

The driver may clear this fault by manually switching to another mode (with the dial) and returning to this pass-through mode.

### Latency Measurement:
- Latency is measured by the difference in times in received messages from drivebrain over CAN. If the difference is too great, we swap to MODE0 control. The
transition delay is negligible.

### Configuration (Defined on construction)
- Maximum allowed latency (milliseconds)
- Assigned controller mode of the drivebrain control mode (currently defaults to MODE4)