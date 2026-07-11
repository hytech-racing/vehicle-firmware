#ifndef VCR_INVERTERS_H
#define VCR_INVERTERS_H

/*
 * This file exists purely because the four inverters (FL/FR/RL/RR) are
 * referenced from two otherwise-independent files that shouldn't depend
 * on each other:
 *
 *   - VCR_InterfaceTasks needs them to build CANInterfacesInstance
 *     and drive other low-level tasks.
 *   - VCR_SystemTasks needs them for the drivetrain funct lambdas
 *     (set_speed/set_idle/get_status/get_motor_mechanics)
 *
 * Without this file, VCR_SystemTasks would have to include VCR_InterfaceTasks.h
 * just to see these four objects. This is undesired because this would also pull in every task
 * function and VCR_InterfaceTasks declares, for something
 * VCR_SystemTasks has no business knowing about.
 *
 * This header only declares (extern) the objects. It does not define
 * the InverterInterface class itself. That still lives in InverterInterface.h.
 */

#include "VCR_Constants.h"

/* Local Interface Includes */
#include "InverterInterface.h"

extern InverterInterface fl_inverter_interface;
extern InverterInterface fr_inverter_interface;
extern InverterInterface rl_inverter_interface;
extern InverterInterface rr_inverter_interface;

#endif