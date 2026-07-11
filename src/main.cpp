#include "Dash_Constants.h"
#include "Dash_Globals.h"
#include "Dash_Tasks.h"
#include "SysClock_Config.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler Setup */
HT_SCHED::Scheduler &scheduler = HT_SCHED::Scheduler::getInstance();

bool spi_tx_complete = true;


/* Task Declarations */
HT_TASK::Task can_task(HT_TASK::DUMMY_FUNCTION, &can_read, 80, 10000); // 10 ms period
// HT_TASK::Task neopixels_task(HT_TASK::DUMMY_FUNCTION, &run_update_neopixels_task, DashConstants::NEOPIXEL_UPDATE_PRIORITY, DashConstants::NEOPIXEL_UPDATE_PERIOD_US);
HT_TASK::Task refresh_screen_task(HT_TASK::DUMMY_FUNCTION, &screen_refresh, DashConstants::SCREEN_REFRESH_PRIORITY, DashConstants::SCREEN_REFRESH_PERIOD_US); // 100 ms period


void setup()
{

    scheduler.setTimingFunction(micros);
    scheduler.schedule(can_task);
    // scheduler.schedule(run_update_neopixels_task);
    scheduler.schedule(refresh_screen_task);

    spi_tx_complete = true;
}

void loop()
{
    scheduler.run();

    //Serial.println(ACUInterfaceInstance::instance().get_curr_data().pack_voltage);
    //Serial.println(VCFInterfaceInstance::instance().get_curr_data().stamped_pedals.pedals_data.brake_percent);
}

