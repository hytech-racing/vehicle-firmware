#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H
#include <stdint.h>

/**
 * @file shared_types.h
 * @brief Types shared ONLY between InverterInterface and DrivetrainSystem,
 * within VCR. This file is intentionally kept separate from
 * SharedFirmwareTypes.h — do not move its contents there, and do not add
 * new types here unless they satisfy the same constraint below.
 *
 * Two independent reasons this file exists, both of which would break if
 * merged into SharedFirmwareTypes.h:
 *
 * 1) SCOPE: shared_types.h is VCR-local. SharedFirmwareTypes.h is compiled
 *    into every board in the monorepo (VCF, ACU, etc.), not just VCR. Types
 *    here (InverterStatus_s, MotorMechanics_s, ...) only mean something in
 *    the context of VCR's own inverters — no other board has any reason to
 *    see them. Moving them into SharedFirmwareTypes.h would make every other
 *    board's build compile against VCR-inverter-specific types for no
 *    reason, and would make SharedFirmwareTypes.h's actual audience (the
 *    whole monorepo) harder to reason about.
 *
 * 2) LAYERING: DrivetrainSystem.h must never #include InverterInterface.h,
 *    and InverterInterface.h must never need to #include DrivetrainSystem.h
 *    or SharedFirmwareTypes.h's drivetrain-level types (DrivetrainCommand_s,
 *    DrivetrainState_e, etc.) beyond the one narrow exception noted on
 *    InverterInterface::reported_mode_matches. This lets DrivetrainSystem
 *    stay fully agnostic to which inverter vendor/protocol is in use (swap
 *    InverterInterface for a different implementation, or a test mock, with
 *    zero changes to DrivetrainSystem), and keeps InverterInterface free of
 *    unrelated drivetrain-level concepts it has no business knowing about.
 *    Everything in this file is deliberately small and generic enough
 *    (bools, floats, no vendor-specific detail) to be safely visible to both
 *    sides without violating that boundary.
 *
 * Before adding a type here, check: does it need to be visible to both
 * InverterInterface.cpp and DrivetrainSystem.cpp specifically, and is it
 * free of any vendor-specific (DTI-specific) detail? If not, it probably
 * belongs somewhere else — either fully inside InverterInterface.h/.cpp (if
 * it's DTI-specific), or in SharedFirmwareTypes.h (if it's a genuine
 * drivetrain-level or monorepo-wide concept unrelated to the inverter
 * boundary specifically).
 */

namespace HTUnits
{
    using celcius = float;
    using watts = float;
    using torque_nm = float;
    using speed_rpm = float;
    using volts = float;
};

struct InverterStatus_s
{
    bool is_inverter_connected;
    bool is_drive_enabled;
    bool is_fault_code_present;
    bool is_hv_present;
    HTUnits::volts dc_bus_voltage;
    unsigned long last_recv_millis = 0;
};

struct MotorMechanics_s
{
    HTUnits::watts actual_power_watts;
    HTUnits::torque_nm actual_torque_nm;
    HTUnits::speed_rpm actual_speed_rpm;
    unsigned long last_recv_millis = 0;
};

#endif // SHARED_TYPES_H