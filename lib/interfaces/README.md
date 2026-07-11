## Inverter Interface

### GPIO connections

### CAN parameter / message descriptions

#### VCR output / control messages:

below is the description of the control messages that will be configured within the free CAN configuration of the AMKs. these are subject to change, however it is primarily a modification of the "fixed CAN commnicaiton" protocol that the AMK kit ships with / is documented 

- Control Word (will be sent every 50ms, monitored by the inverter to be expected at least every 75ms)
    - `bool inverter_enable` 
        - AKA (`AMK_bInverterOn`)
    - `bool hv_enable` 
        - AKA (`AMK_bDcOn`)
    - `bool driver_enable`
        - AKA (`AMK_bEnable`)
    - `bool remove_error` 
        - AKA (`AMK_bErrorReset`)

- Control Input Message (will be attempted to be sent every 5ms, monitored by the inverter to be expected at least every 20ms)
    - `int16_t speed_setpoint` 
        - __NOTE: active only when in speed control mode__
        - AKA (`AMK_TargetVelocity`) / Special Signal index 6 / AKA 16-bit version of SERCOS parameter ID36 (unknown why this is 16 bit vs the 32 bit SERCOS parameter in manual)
        - the RPM setpoint for the inverter in units of RPM.
    - `int16_t positive_torque_limit` 
        - AKA (`AMK_TorqueLimitPositiv`) / SERCOS parameter ID82 / Special Signal index 13
        - positive torque limit for reaching the desired rpm in units of 0.1% of the nominal torque (=9.8Nm)(known as Mn in docs). 

            EXAMPLE: `positive_torque_limit=1000` means that the positive torque limit is set to 100% of the nominal torque, so the actual torque limit being set by this is equal to 9.8Nm, and 200% would be equal to 19.6Nm.
    - `int16_t negative_torque_limit`
        - AKA (`AMK_TorqueLimitNegativ`) / SERCOS parameter ID83 / Special Signal index 14
        - `positive_torque_limit` with a negative sign.
    - `int16_t torque_setpoint` 
        - __NOTE: active only when in torque control mode.__
        - AKA SERCOS parameter ID80 / Special Signal index 17
        - the torque setpoint for the inverter in units of percentage as described in the `positive_torque_limit` message member above

- Control parameter message (will be sent intermitently, not monitored by the inverter to be expected at any regular period)
    - `uint16_t speed_control_kp`
        - P for the speed control mode
        - AKA SERCOS parameter ID100
    - `uint16_t speed_control_ki`
        - I parameter to be set internally on the inverter
        - AKA SERCOS parameter ID101
    - `uint16_t speed_control_kd`
        - D parameter to be set internally on the inverter
        - AKA SERCOS parameter ID102

#### inverter messages

- inverter status / ( `AMK_Status` (16 bit) +  + DC bus voltage (16 bit) + `AMK_ErrorInfo` (16 bit kinda(?))) (will be periodically sent from the inverter every 20ms)
    - `bool system_ready`
        - AKA (`AMK_bSystemReady`) / bit 9 within Special Signal Status word formula student (System Ready (SBM)) / 
        - displays when the system is error free / ready to be initialized
    - `bool error`
        - AKA (`AMK_bError`) / bit 10 within Special Signal Status word formula student 
        - displays that an error is present
    - `bool warning`
        - AKA (`AMK_bWarn`) / bit 11
        - warning present (such as if derating is on)
    - `bool quit_dc_on`
        - AKA (`AMK_bQuitDcOn`) / bit 12
        - HV activation acknowledgment / verification (says whether or not HV is ACTUALLY present to the inverter)
    - `bool dc_on`
        - AKA (`AMK_bDcOn`) / bit 13
        - mirrors / is feedback of what is set by the VCR for its `hv_enable`
    - `bool quit_inverter_on`
        - AKA (`AMK_bQuitInverterOn`) / bit 14
        - controller is enabled
    - `bool inverter_on`
        - AKA (`AMK_bInverterOn`) / bit 15
        - mirror / feedback of `inverter_enable` set by the control word
    - `bool derating_on`
        - AKA (`AMK_bDerating`) / bit 16
        - says whether or not derating is active (torque derating)
    - `uint16_t dc_bus_voltage`
        - AKA SERCOS Paramter ID32836
        - actual DC bus voltage 
    - `uint16_t diagnostic_number`
        - AKA (`AMK_ErrorInfo`)

- inverter temps (will be periodically sent from the inverter every 20ms)
    - `int16_t motor_temp`
        - temperature of the motor in units of 0.1 degrees celcius
        - AKA (`AMK_TempMotor`) / Special Signal index 7 / SERCOS parameter ID33117 
    - `int16_t inverter_temp`
        - temp of the inverter cold plate in units of 0.1 degrees celcius
        - AKA (`AMK_TempInverter`) / Special Signal index 8 / SERCOS parameter ID33116

            - details: "ID33116 shows the temperature of the cold plate (heat sink of the IGBT and at the same time of the rear wall of the device).
The triggering thresholds are device-specific, are set in the SEEP (Device-internal memory, serial EEPROM) at the factory and cannot be changed by the user.
If critical temperatures occur for the devices, the warning 2350 'Device temperature warning' is generated as well as the error
message 2346 'Converter temperature error' after the warning time1) (ID32943) has expired."
    - `int16_t igbt_temp`
        - temp of the IGBTs in units of 0.1 degrees celcius
        - AKA (`AMK_TempIGBT`) / Special Signal index 27 / SERCOS parameter ID34215

- inverter dynamics data (will be periodically sent from the inverter every 5ms)
    - `uint32_t actual_power_w`
        - mechanical motor power in watts (from actual torque and actual speed)
        - AKA SERCOS parameter ID33100
    - `int16_t actual_torque_nm`
        - actual torque value in the same units as `positive_torque_limit` 
        - AKA Special Signal index 19 / SERCOS parameter ID84
    - `int16_t actual_speed_rpm`
        - motor speed in units of rpm
        - AKA Special Signal value index 6

- inverter electrical power data (will be periodically sent from the inverter every 5ms)
    - `int32_t active_power_w`
        - electrical power (use / creation) in watts of the inverter
        - can be negative when in regen / positive when driving motor
        - AKA SERCOS parameter ID33171
    - `int32_t reactive_power_var` 
        - electrical reactive power (inductive or capacitive)
        - positive value = inductive consumer, negative value = capacitive consumer
        - AKA SERCOS parameter ID33172
    
- inverter parameter feedback (inverter's speed PID vals) (will be sent periodically from the inverter every 20ms)
    - `uint16_t speed_control_kp`
        - inverter's internal value of kp
        - AKA SERCOS parameter ID100
    - `uint16_t speed_control_ki`
        - inverter's internal value of (TN) ki
        - AKA SERCOS parameter ID101
    - `uint16_t speed_control_kd`
        - inverter's internal value of (TD) kd
        - AKA SERCOS parameter ID102

## VCF Interface

The VCR is connected over CAN and Ethernet to the VCF. We will use the CAN communication for latency-sensitive communication such as the driver input and controller input sensor signals. The Ethernet link will be used for the non-timing-sensitive data.

### CAN interface

VCF Outputs:
- pedal data CAN packet (`0xC0`):
    - status bits: (8 bits)
        - `bool accel_implausible`
            - accel pedal value is out of range
        - `bool brake_implausible`
            - brake pedal value is out of range
        - `bool brake_pedal_active`
        - `bool accel_pedal_active`
        - `bool mech_brake_active`
            - brake pedal has reached zone in which the mechanical brake (the physical calipers) have started engaging
        - `bool brake_and_accel_pressed_implausibility`
        - `bool implausibility_exceeded_duration`
            - an implausibility been present for longer than allowed (>200ms by rules)
                - __note__: we should guard this to be over 180ms or some threshold below 200ms to allow for transmission delay to stay within rules as this is now being reacted to by the VCR
    - data (32 bits):
        - `brake` (16 bit unsigned) -> mapped between 0 and 1 (65,535)
        - `accel` (16 bit unsigned) -> mapped between 0 and 1 (65,535)

    - __note__: the regen percentage that was present on MCU should instead be calculated by the controllers themselves instead of by the pedals system to centralize regen calculation at higher levels to allow for tuning / safe modification more easily

- __note__: the following data is all the raw, non-filtered data
    - `STEERING_DATA` (`0x41f`) steering data CAN packet:
        - `uint16_t steering_analog_raw`
        - `float steering_digital_raw` (32 bit) 


### Ethernet Interface

`VCFOutputData`
VCF Outputs:
- user inputs
    - requested drive mode (0 through 5)
    - requesting drivebrain / VCR in control mode
    - requested torque limit mode
    - requesting drivetrain error reset
- statuses:
    - buzzer status
- info:
    - firmware version info
        - `bool dirty`
            - means that the firmware was flashed while there was changes made that had not been commited
        - `char git_short_hash[8]`
            - the git hash of the commit that was flashed to the 

## Drivebrain Interface requirements

### CAN interface requirements

#### preface
__NOTE__: the following messages that are on the bus are listened to by the drivebrain and are output from other boards (not VCR)
- pedal data CAN packet (VCF)
- inverter data (FL, FR, RL, RR inverters) -> these will be forwarded messages being forwarded by VCR onto the telem CAN line

VCR Outputs:

- `REAR_SUSPENSION` (`0E4`) VCR suspension data CAN packet: (200hz)
    - `uint16_t rl_load_cell`
    - `uint16_t rr_load_cell`
    - `uint16_t rl_shock_pot`
    - `uint16_t rr_shock_pot`

- VCR status CAN packet: (5hz)
    - `uint8_t vehicle_state_index`: vehicle state enum index (oneof: RTD, tractive system enabled, etc.)
    - `uint8_t control_mode_index` control mode index (MODE0 through MODE5)
    - `uint16_t vcr_error_state_word` VCR error state word 
        - bit 1: drivebrain timeout present
        - bit 2: VCR firmware error
        - bit 3: drivetrain error present
        - bits 4 through 16: (reserved)

VCR inputs (sent by drivebrain and VCF):
- `DRIVEBRAIN_SPEED_SET_INPUT`: (`0xF2`) desired RPMs
    - `uint16_t drivebrain_set_rpm_fl`
    - `uint16_t drivebrain_set_rpm_fr`
    - `uint16_t drivebrain_set_rpm_rl`
    - `uint16_t drivebrain_set_rpm_rr`

- `DRIVEBRAIN_TORQUE_LIM_INPUT`: (`0xF1`) torque limits (de-facto torque setpoint in certain conditions)
    - __note__: in units of .01nm: 2100 = 21nm, 2150 = 21.5nm
    - `int16_t drivebrain_torque_fl_torque` 
    - `int16_t drivebrain_torque_fr_torque`
    - `int16_t drivebrain_torque_rl_torque`
    - `int16_t drivebrain_torque_rl_torque`



