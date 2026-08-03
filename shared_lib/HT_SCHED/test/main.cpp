#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <stdio.h>
#include <functional>
#include "ht_sched.hpp"

auto start_time = std::chrono::high_resolution_clock::now();

uint32_t stdMicros()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    return static_cast<uint32_t>(elapsed);
}

HT_TASK::TaskResponse task1F(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    std::cout << "task1 exec " << taskInfo.executions << "\n";
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse task2F(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    std::cout << "task2 exec " << taskInfo.executions << "\n";
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse task3F(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // std::cout << "task3 filtered duration " << taskInfo.filteredExecutionDurationMicros << "\n";
    // std::cout << "task3 last execution " << taskInfo.lastExecutionMicros << "\n";
    return HT_TASK::TaskResponse::YIELD;
}

HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

HT_TASK::Task task1 = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, task1F, 2, 20000UL); // 20000us is 50hz
HT_TASK::Task task2 = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, task2F, 1, 20000UL); // task2 is higher priority
HT_TASK::Task task3 = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, task3F, 0); // task3 is an idle task

HT_TASK::Task schedMon = HT_TASK::Task(
    std::bind(&HT_SCHED::Scheduler::initSchedMon, std::ref(scheduler), std::placeholders::_1, std::placeholders::_2), 
    std::bind(&HT_SCHED::Scheduler::schedMon, std::ref(scheduler), std::placeholders::_1, std::placeholders::_2), 
    100000UL,
    0
);

int main(void)
{
    scheduler.setTimingFunction(stdMicros);
    scheduler.schedule(task1);
    scheduler.schedule(task2);
    scheduler.schedule(task3);
    scheduler.schedule(schedMon);

    while (std::chrono::high_resolution_clock::now() - start_time < std::chrono::seconds(1))
    {
        scheduler.run();
    }

    return 0;
}