#include "CCU_Constants.hpp"
#include "CCU_InterfaceTasks.hpp"
#include "CCU_SystemTasks.hpp"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Systems */
namespace qn = qindesign::network; // setup of qn namespace
qn::EthernetUDP udp; // setup of qn namespace

/* Scheduler Setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
/* read_dial, send_ethernet, and receieve_ethernet are not being used */
HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, &kickWatchdogTask, CCUConstants::KICK_WATCHDOG_PRIORITY, CCUConstants::KICK_WATCHDOG_PERIOD);
HT_TASK::Task run_sample_can_data(HT_TASK::DUMMY_FUNCTION, &sampleCANTask, CCUConstants::SAMPLE_CAN_DATA_PRIORITY, CCUConstants::SAMPLE_CAN_DATA_PERIOD);
HT_TASK::Task tick_state_machine_task(HT_TASK::DUMMY_FUNCTION, &tickStateMachineTask, CCUConstants::TICK_STATE_MACHINE_PRIORITY, CCUConstants::TICK_STATE_MACHINE_PERIOD);
HT_TASK::Task calculate_charge_current_task(HT_TASK::DUMMY_FUNCTION, &calculateChargeCurrentTask, CCUConstants::TICK_STATE_MACHINE_PRIORITY, CCUConstants::TICK_STATE_MACHINE_PERIOD);
HT_TASK::Task queue_ACU_CAN(HT_TASK::DUMMY_FUNCTION, &enqueueACUCANDataTask, CCUConstants::ENQUEUE_ACU_CAN_DATA_PRIORITY, CCUConstants::ENQUEUE_ACU_CAN_DATA_PERIOD);
HT_TASK::Task queue_Charger_CAN(HT_TASK::DUMMY_FUNCTION, &enqueueChargerCANdataTask, CCUConstants::ENQUEUE_CHARGER_CAN_DATA_PRIORITY, CCUConstants::ENQUEUE_CHARGER_CAN_DATA_PERIOD);
HT_TASK::Task send_all_data(HT_TASK::DUMMY_FUNCTION, &clearAllCANBuffersTask, CCUConstants::SEND_ALL_DATA_PRIORITY, CCUConstants::SEND_ALL_DATA_PERIOD);
HT_TASK::Task receive_ethernet(HT_TASK::DUMMY_FUNCTION, &receiveETHTask, CCUConstants::RECIEVE_ETHERNET_PRIORITY, CCUConstants::ETHERNET_PERIOD);
HT_TASK::Task send_ethernet(HT_TASK::DUMMY_FUNCTION, &sendETHTask, CCUConstants::SEND_ETHERNET_PRIORITY, CCUConstants::ETHERNET_PERIOD);
HT_TASK::Task read_encoder_task(HT_TASK::DUMMY_FUNCTION, &readEncoderTask, CCUConstants::READ_DIAL_PRIORITY, CCUConstants::DIAL_PERIOD);
HT_TASK::Task update_display_task(HT_TASK::DUMMY_FUNCTION, &updateDisplayTask, CCUConstants::UPDATE_DISPLAY_PRIORITY, CCUConstants::UPDATE_DISPLAY_PERIOD);
HT_TASK::Task debug_print_task(HT_TASK::DUMMY_FUNCTION, &debug_prints, CCUConstants::UPDATE_DISPLAY_PRIORITY, CCUConstants::UPDATE_DISPLAY_PERIOD);

void setup()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(CCUInterfaces::DISPLAY_BAUDRATE, MSBFIRST, SPI_MODE0)); //NOLINT (spi settings)

    qn::Ethernet.begin();

    initializeAllInterfaces();
    initializeAllSystems();

    scheduler.setTimingFunction(micros);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(run_sample_can_data);
    scheduler.schedule(tick_state_machine_task);
    scheduler.schedule(calculate_charge_current_task);
    scheduler.schedule(queue_ACU_CAN);
    scheduler.schedule(queue_Charger_CAN);
    scheduler.schedule(send_all_data);
    scheduler.schedule(receive_ethernet);
    scheduler.schedule(send_ethernet);
    scheduler.schedule(read_encoder_task);
    // scheduler.schedule(debug_print_task);
    scheduler.schedule(update_display_task);
}

void loop()
{
    scheduler.run();
}

