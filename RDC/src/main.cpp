#include <Arduino.h>

#include "RDC_Constants.h"
#include "RDC_Tasks.h"
#include "SharedFirmwareTypes.h"

/* Scheduler Setup */
HT_SCHED::Scheduler &scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
HT_TASK::Task send_can_task(HT_TASK::DUMMY_FUNCTION, &handleSendAllCanData, RDCConstants::SEND_CAN_PRIORITY, RDCConstants::SEND_CAN_PERIOD_US);
HT_TASK::Task enqueue_fdc_control_task(HT_TASK::DUMMY_FUNCTION, &enqueueFDCControlDVState, RDCConstants::ENQUEUE_FDC_CONTROL_PRIORITY, RDCConstants::ENQUEUE_FDC_CONTROL_PERIOD_US);
HT_TASK::Task enqueue_rss_operating_mode_task(HT_TASK::DUMMY_FUNCTION, &enqueueRSSOperatingMode, RDCConstants::ENQUEUE_RSS_OPERATING_MODE_PRIORITY, RDCConstants::ENQUEUE_RSS_OPERATING_MODE_PERIOD_US);

HT_TASK::Task run_tick_state_machine_task(HT_TASK::DUMMY_FUNCTION, &runTickStateMachine, RDCConstants::TICK_STATE_MACHINE_PRIORITY, RDCConstants::TICK_STATE_MACHINE_PERIOD_US);
HT_TASK::Task collect_async_data_task(HT_TASK::DUMMY_FUNCTION, &handleAsyncMain, RDCConstants::COLLECT_ASYNC_DATA_PRIORITY, RDCConstants::COLLECT_ASYNC_DATA_PERIOD_US);
HT_TASK::Task kick_watchdog_task(&initWatchdog, &runKickWatchdog, RDCConstants::KICK_WATCHDOG_PRIORITY, RDCConstants::KICK_WATCHDOG_PERIOD_US);

HT_TASK::Task debug_print_task(HT_TASK::DUMMY_FUNCTION, &debugPrint, RDCConstants::DEBUG_PRINT_PRIORITY, RDCConstants::DEBUG_PRINT_PERIOD_US);
void setup()
{
    initializeAllInterfaces();
    initializeDriverless();

    scheduler.setTimingFunction(micros);

    scheduler.schedule(send_can_task);
    scheduler.schedule(enqueue_fdc_control_task);
    scheduler.schedule(enqueue_rss_operating_mode_task);

    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(run_tick_state_machine_task);
    scheduler.schedule(collect_async_data_task);

    scheduler.schedule(debug_print_task);
}

void loop()
{
    scheduler.run();
}
