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
HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, &kickWatchdogTask, VCFConstants::WATCHDOG_KICK_PERIOD_US, VCFConstants::WATCHDOG_PRIORITY);
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, VCFConstants::ASYNC_MAIN_PERIOD_US, VCFConstants::ASYNC_MAIN_PRIORITY);
HT_TASK::Task adc0_sample(HT_TASK::DUMMY_FUNCTION, &readADC0Task, VCFConstants::PEDALS_SAMPLE_PERIOD_US, VCFConstants::PEDALS_SAMPLE_PRIORITY);
HT_TASK::Task adc1_sample(HT_TASK::DUMMY_FUNCTION, &readADC1Task, VCFConstants::LOADCELL_SAMPLE_PERIOD_US, VCFConstants::LOADCELL_SAMPLE_PRIORITY);
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

    initializeAllInterfaces();
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


namespace async_tasks
{
    // these are async tasks. we want these to run as fast as possible p much
    void handle_async_CAN_receive() //NOLINT caps for CAN
    {
        process_ring_buffer(VCFCANInterfaceInstance::instance().telem_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), VCFCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::TELEM);
        process_ring_buffer(VCFCANInterfaceInstance::instance().front_aux_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), VCFCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::FAUX);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...

        handle_async_CAN_receive();
    }

    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();

        SteeringSystemInstance::instance().evaluate_steering(
            ADCInterfaceInstance::instance().getSteeringDegreesCW().conversion,
            OrbisInterfaceInstance::instance().getLastReading(),
            sys_time::hal_millis()
        );

        PedalsSystemInstance::instance().evaluate_pedals(
            PedalsSystemInstance::instance().get_pedals_sensor_data(),
            sys_time::hal_millis()
        );
        return HT_TASK::TaskResponse::YIELD;
    }
};

HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    /* Pedals Info */
    Serial.println("\n\nPedals Info:");
    Serial.println("\tPercent Pressed Implaus Min 1 \tMax 1 \tMin 2 \tMax 2");
    // Accel
    Serial.print("Accel: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
    // Brake
    Serial.print("Brake: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_brake_params().max_pedal_2);

    /* Steering System Data */
    Serial.println("Steering Sensor Data: ");
    Serial.print("analog adc: ");
    Serial.print(SteeringSystemInstance::instance().get_steering_system_data().analog_raw); Serial.print(" ");
    Serial.print(ADCInterfaceInstance::instance().getSteeringDegreesCW().raw);
    Serial.print("|");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_angle);
    Serial.print("digital adc: ");
    Serial.print(SteeringSystemInstance::instance().get_steering_system_data().digital_raw);
    Serial.print("|");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_angle);
    Serial.print("min_observed_analog: ");
    // Serial.println(SteeringSystemInstance::instance().get_min_observed_analog());
    // Serial.print("max_observed_analog: ");
    // Serial.println(SteeringSystemInstance::instance().get_max_observed_analog());
    Serial.print("analog_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_angle);
    Serial.print("digital_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_angle);
    Serial.print("time: ");
    Serial.println(sys_time::hal_millis());

    Serial.print("output_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().output_steering_angle);

    Serial.print("analog_steering_velocity_deg_s: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_velocity_deg_s);
    Serial.print("digital_steering_velocity_deg_s: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_velocity_deg_s);

    Serial.print("digital_oor_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_oor_implausibility);
    Serial.print("analog_oor_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_oor_implausibility);
    Serial.print("sensor_disagreement_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().sensor_disagreement_implausibility);
    Serial.print("dtheta_exceeded_analog: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().dtheta_exceeded_analog);
    Serial.print("dtheta_exceeded_digital: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().dtheta_exceeded_digital);
    Serial.print("both_sensors_fail: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().both_sensors_fail);
    Serial.print("interface_sensor_error: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().interface_sensor_error);

    /* ADC Values */
    Serial.println("\nADC Vals:");
    // ADC 0
    Serial.println("ADC 0\t\t  Steering");
    Serial.println("\t2V5 Ref CW \tCCW \tAccel 1 Accel 2 Brake 1 Brake 2");
    // Raw values
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().getPedalReference().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getSteeringDegreesCW().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getSteeringDegreesCCW().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getAcceleration1().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getAcceleration2().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getBrake1().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().getBrake2().raw);
    // Converted values
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().getPedalReference().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getSteeringDegreesCW().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getSteeringDegreesCCW().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getAcceleration1().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getAcceleration2().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getBrake1().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().getBrake2().conversion);

    // ADC 1
    Serial.println("\nADC 1\t\t\t  Load Cells \t  Sus Pots \t Brake Pressure");
    Serial.println("\tSHDN H \tSHDN D \tFL \tFR \tFR \tFL \tFront \tRear");
    // Raw ADC
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().getShutdownH().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getShutdownD().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFLLoadcell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFRLoadcell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFRSuspot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFLSuspot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getBrakePressureFront().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().getBrakePressureRear().raw);
    // Conversion ADC
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().getShutdownH().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getShutdownD().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFilteredFLLoadcell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFilteredFRLoadcell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFilteredFRSuspot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getFilteredFLSuspot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().getBrakePressureFront().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().getBrakePressureRear().conversion);

    /* Dashboard Info */
    Serial.println("\nDash Buttons / Buzzer:");
    Serial.println("Preset \tReset \tStart \tData \tBuzzer");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().preset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().mc_reset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().start_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().data_btn_is_pressed); Serial.print("\t");
    Serial.println(BuzzerControllerInstance::instance().isBuzzerActive(sys_time::hal_millis()));

    /* Brake Rotor Temp Info */
    Serial.println("\nBrake Rotor Temps:");
    Serial.println("Sensor\tMax\tAvg\tCH0\tCH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8\tCH9\tCH10\tCH11\tCH12\tCH13\tCH14\tCH15");

    // Sensor 1
    Serial.print("FL\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.max_temp); Serial.print("\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.avg_temp); Serial.print("\t");

    for (size_t i = 0; i < brake_rotor_temp_default_params::channels_within_brake_temp_sensor; ++i)
    {
        Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.channel_data[i]);
        Serial.print("\t");
    }
    Serial.println();

    // Sensor 2
    Serial.print("FR\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.max_temp); Serial.print("\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.avg_temp); Serial.print("\t");

    for (size_t i = 0; i < brake_rotor_temp_default_params::channels_within_brake_temp_sensor; ++i)
    {
        Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.channel_data[i]);
        Serial.print("\t");
    }
    Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}