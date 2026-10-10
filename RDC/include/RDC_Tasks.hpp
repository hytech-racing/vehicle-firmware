#ifndef RDC_INTERFACETASKS_H
#define RDC_INTERFACETASKS_H

#include "RDC_Constants.hpp"

/* External Includes */
#include "ht_sched.hpp"

/* Local Interface Includes */
#include "ADCInterface.hpp"
#include "CoreRDCCANInterface.hpp"
#include "RSSInterface.hpp"
#include "SystemTimeInterface.h"
#include "ht_task.hpp"

/* System Includes */
#include "DriverlessStateMachine.hpp"
#include "DriverlessSystem.hpp"
#include "WatchdogSystem.hpp"

void initializeAllInterfaces();
void initializeDriverless();

/* CAN Tasks */
HT_TASK::TaskResponse handleSendAllCanData(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
HT_TASK::TaskResponse enqueueFDCControlDVState(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
HT_TASK::TaskResponse enqueueRSSOperatingMode(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);

/* State Machine Tasks */
HT_TASK::TaskResponse initTickStateMachine(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
HT_TASK::TaskResponse runTickStateMachine(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);

/* Background Tasks */
HT_TASK::TaskResponse handleAsyncMain(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
HT_TASK::TaskResponse initWatchdog(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
HT_TASK::TaskResponse runKickWatchdog(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);

HT_TASK::TaskResponse debugPrint(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info);
#endif
