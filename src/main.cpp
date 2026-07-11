#include "VCF_Constants.h"
#include "VCF_InterfaceTasks.h"
#include "VCF_SystemTasks.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Systems */
namespace qn = qindesign::network; // setup of qn namespace
qn::EthernetUDP udp; // setup of qn namespace

/* Scheduler Setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, &run_kick_watchdog, VCFConstants::WATCHDOG_KICK_PERIOD_US, VCFConstants::WATCHDOG_PRIORITY);
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, VCFConstants::ASYNC_MAIN_PERIOD_US, VCFConstants::ASYNC_MAIN_PRIORITY);
HT_TASK::Task adc0_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc0_task, VCFConstants::PEDALS_SAMPLE_PERIOD_US, VCFConstants::PEDALS_SAMPLE_PRIORITY);
HT_TASK::Task adc1_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc1_task, VCFConstants::LOADCELL_SAMPLE_PERIOD_US, VCFConstants::LOADCELL_SAMPLE_PRIORITY);
HT_TASK::Task pedals_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_pedals_data, VCFConstants::PEDALS_SEND_PERIOD_US, VCFConstants::PEDALS_SEND_PRIORITY);
HT_TASK::Task steering_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_steering_data, VCFConstants::STEERING_SEND_PERIOD_US, VCFConstants::STEERING_SEND_PRIORITY);
HT_TASK::Task front_suspension_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_front_suspension_data, VCFConstants::LOADCELL_SEND_PERIOD_US, VCFConstants::LOADCELL_SEND_PRIORITY);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, &handle_CAN_send, VCFConstants::CAN_SEND_PERIOD_US, VCFConstants::CAN_SEND_PRIORITY);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, &send_dash_data, VCFConstants::DASH_SEND_PERIOD_US, VCFConstants::DASH_SEND_PRIORITY);
HT_TASK::Task read_dash_GPIOs_task(HT_TASK::DUMMY_FUNCTION, &run_dash_GPIOs_task, VCFConstants::DASH_SAMPLE_PERIOD_US, VCFConstants::DASH_SAMPLE_PRIORITY);
HT_TASK::Task ethernet_send_task(init_handle_send_vcf_ethernet_data, run_handle_send_vcf_ethernet_data, VCFConstants::ETHERNET_SEND_PERIOD_US, VCFConstants::ETHERNET_SEND_PRIORITY);
HT_TASK::Task buzzer_control_task(&init_buzzer_control_task, &run_buzzer_control_task, VCFConstants::BUZZER_WRITE_PERIOD_US, VCFConstants::BUZZER_PRIORITY);
HT_TASK::Task neopixels_task(HT_TASK::DUMMY_FUNCTION, &update_neopixels_task, VCFConstants::NEOPIXEL_UPDATE_PERIOD_US, VCFConstants::NEOPIXEL_UPDATE_PRIORITY);
HT_TASK::Task pedals_calibration_task(HT_TASK::DUMMY_FUNCTION, &update_pedals_calibration_task, VCFConstants::PEDALS_RECALIBRATION_PERIOD_US, VCFConstants::PEDALS_RECALIBRATION_PRIORITY);
HT_TASK::Task steering_calibration_task(HT_TASK::DUMMY_FUNCTION, &update_steering_calibration_task, VCFConstants::STEERING_RECALIBRATION_PERIOD_US, VCFConstants::STEERING_RECALIBRATION_PRIORITY);
HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, &debug_print, VCFConstants::DEBUG_PERIOD_US, VCFConstants::DEBUG_PRIORITY);

void setup()
{
    qn::Ethernet.begin();

    initialize_all_interfaces();
    initialize_all_systems();


    // Setup scheduler
    HT_SCHED::Scheduler::getInstance().setTimingFunction(micros);

    // Schedule Tasks
    HT_SCHED::Scheduler::getInstance().schedule(kick_watchdog_task);
    HT_SCHED::Scheduler::getInstance().schedule(async_main);
    HT_SCHED::Scheduler::getInstance().schedule(CAN_send);
    HT_SCHED::Scheduler::getInstance().schedule(dash_CAN_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(buzzer_control_task);
    HT_SCHED::Scheduler::getInstance().schedule(pedals_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(adc0_sample);
    HT_SCHED::Scheduler::getInstance().schedule(adc1_sample);
    HT_SCHED::Scheduler::getInstance().schedule(read_dash_GPIOs_task);
    HT_SCHED::Scheduler::getInstance().schedule(neopixels_task);
    HT_SCHED::Scheduler::getInstance().schedule(steering_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(front_suspension_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(pedals_calibration_task);
    HT_SCHED::Scheduler::getInstance().schedule(steering_calibration_task);
    HT_SCHED::Scheduler::getInstance().schedule(ethernet_send_task);

    HT_SCHED::Scheduler::getInstance().schedule(debug_state_print_task);
}

void loop() {
    HT_SCHED::Scheduler::getInstance().run();
}
