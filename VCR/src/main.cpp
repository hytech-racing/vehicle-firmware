#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_InterfaceTasks.h"
#include "VCR_SystemTasks.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Systems */
namespace qn = qindesign::network; // setup of qn namespace
qn::EthernetUDP udp; // setup of qn namespace

/* Scheduler Setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, &run_kick_watchdog, VCRConstants::WATCHDOG_PRIORITY, VCRConstants::WATCHDOG_KICK_PERIOD_US);
HT_TASK::Task async_main_task(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, VCRConstants::ASYNC_MAIN_PRIORITY, VCRConstants::ASYNC_MAIN_PERIOD_US);
HT_TASK::Task send_CAN_task(HT_TASK::DUMMY_FUNCTION, handle_send_all_CAN_data, VCRConstants::SEND_CAN_PRIORITY, VCRConstants::SEND_CAN_PERIOD_US); // Sends all messages from the CAN queue
HT_TASK::Task ams_system_task(HT_TASK::DUMMY_FUNCTION, update_acu_heartbeat, VCRConstants::AMS_PRIORITY, VCRConstants::ASYNC_MAIN_PERIOD_US);
HT_TASK::Task enqueue_suspension_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_suspension_CAN_data, VCRConstants::SUSPENSION_PRIORITY, VCRConstants::SUSPENSION_CAN_PERIOD_US);
HT_TASK::Task enqueue_inverter_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_inverter_CAN_data, VCRConstants::INVERTER_SEND_PRIORITY, VCRConstants::INVERTER_SEND_PERIOD_US);
HT_TASK::Task vcr_data_ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, VCRConstants::ETHERNET_SEND_PRIORITY, VCRConstants::ETHERNET_SEND_PERIOD_US);
HT_TASK::Task adc_0_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc0_task, VCRConstants::ADC0_PRIORITY, VCRConstants::ADC0_SAMPLE_PERIOD_US);
HT_TASK::Task enqueue_controls_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_controls_CAN_data, VCRConstants::CONTROLS_PRIORITY, VCRConstants::CONTROLS_CAN_PERIOD_US);
HT_TASK::Task enqueue_coolant_temp_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_coolant_temp_CAN_data, VCRConstants::COOLANT_TEMP_SEND_PRIORITY, VCRConstants::COOLANT_TEMP_SEND_PERIOD_US);
HT_TASK::Task enqueue_dashboard_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_dashboard_CAN_data, VCRConstants::DASHBOARD_SEND_PRIORITY, VCRConstants::DASHBOARD_SEND_PERIOD_US);
HT_TASK::Task run_enable_motor_cooling(HT_TASK::DUMMY_FUNCTION, enable_motor_cooling, VCRConstants::DASHBOARD_SEND_PRIORITY, VCRConstants::DASHBOARD_SEND_PERIOD_US);
HT_TASK::Task run_enable_inverter_cooling(HT_TASK::DUMMY_FUNCTION, enable_inverter_cooling, VCRConstants::DASHBOARD_SEND_PRIORITY, VCRConstants::DASHBOARD_SEND_PERIOD_US);
HT_TASK::Task update_brakelight_task(HT_TASK::DUMMY_FUNCTION, run_update_brakelight_task, VCRConstants::UPDATE_BRAKELIGHT_PRIORITY, VCRConstants::UPDATE_BRAKELIGHT_PERIOD_US);
HT_TASK::Task adc_1_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc1_task, VCRConstants::ADC1_PRIORITY, VCRConstants::ADC1_SAMPLE_PERIOD_US);
HT_TASK::Task IOExpander_read_task(HT_TASK::DUMMY_FUNCTION, read_ioexpander, VCRConstants::IOEXPANDER_PRIORITY, VCRConstants::IOEXPANDER_SAMPLE_PERIOD_US);
HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, debug_print, VCRConstants::DEBUG_PRIORITY, VCRConstants::DEBUG_PERIOD_US);

void setup()
{
    // Configure pins
    pinMode(VCRInterfaces::MOTOR_COOLING_CONTROL_PIN, OUTPUT);
    pinMode(VCRInterfaces::INVERTER_COOLING_CONTROL_PIN, OUTPUT);
    pinMode(VCRInterfaces::INVERTER_ENABLE_PIN, OUTPUT);


    // Flowmeter stuff
    // pinMode(FLOWMETER_PIN, INPUT_PULLUP); //need to change based on aux uart
    // pinMode(27, OUTPUT);
    // digitalWrite(27, HIGH);

    //attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), countPulse, RISING);
    //pulseCount = 0;

    // Scheduler timing function
    scheduler.setTimingFunction(micros);

    // Schedule scheduler tasks
    scheduler.schedule(adc_0_sample_task);
    // scheduler.schedule(adc_1_sample_task);

    scheduler.schedule(kick_watchdog_task);

    scheduler.schedule(ams_system_task); // ensure ACU connection
    scheduler.schedule(enqueue_suspension_CAN_task);
    scheduler.schedule(enqueue_dashboard_CAN_task);

    scheduler.schedule(send_CAN_task);

    scheduler.schedule(vcr_data_ethernet_send);

    scheduler.schedule(enqueue_inverter_CAN_task);
    scheduler.schedule(enqueue_coolant_temp_CAN_task);
    scheduler.schedule(async_main_task);

    scheduler.schedule(enqueue_controls_CAN_task);

    // scheduler.schedule(debug_state_print_task);

    scheduler.schedule(update_brakelight_task);

    // scheduler.schedule(update_sample_flowmeter);

    scheduler.schedule(IOExpander_read_task);

    scheduler.schedule(run_enable_motor_cooling);
    scheduler.schedule(run_enable_inverter_cooling);
}

void loop()
{
    scheduler.run();
}
