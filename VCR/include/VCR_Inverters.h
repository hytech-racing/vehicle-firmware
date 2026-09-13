#ifndef VCR_INVERTERS_H
#define VCR_INVERTERS_H

/*
 * The four inverters (FL/FR/RL/RR) are referenced from two otherwise -
 * independent files:
 *
 *   - VCR_InterfaceTasks needs them directly to build CANInterfacesInstance
 *     and drive other low-level CAN tasks.
 *   - VCR_SystemTasks needs a hardware-agnostic bundle of callbacks
 *     (InverterFuncts_s) for the drivetrain system, via make_inverter_functs().
 *
*/

#include "VCR_Constants.h"
#include "InverterInterface.h"
#include "DrivetrainSystem.h"   // only for the InverterFuncts_s return type

extern InverterInterface fl_inverter_interface;
extern InverterInterface fr_inverter_interface;
extern InverterInterface rl_inverter_interface;
extern InverterInterface rr_inverter_interface;

/**
 * @brief Builds the DrivetrainSystem-facing function bundle for all four inverters.
*/
veh_vec<InverterInterfaceFuncts_s> makeInverterFuncts();

#endif