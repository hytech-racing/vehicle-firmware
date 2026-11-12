#include <Arduino.h>
#include "benchmark.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

constexpr std::size_t MAX_TASKS = HT_SCHED_MAX_TASKS;///currently 32
constexpr std::size_t TASK_COUNTS[] = {1, 4, 8, 16, 28, 32};

///Benchmark config
const bench::BenchmarkConfig benchmarkConfig = {
    2000, /// warmup iterations
    3000, /// # of measurements
    1     /// 1 scheduler run per measurement
};

bench::Benchmark benchmark(benchmarkConfig);

HT_TASK::Task tasks[MAX_TASKS];

HT_TASK::TaskResponse noOpTask(const uint32_t& timeMicros, const HT_TASK::TaskInfo& taskInfo)
{
    (void)timeMicros;
    (void)taskInfo;
    return HT_TASK::TaskResponse::YIELD;
}

void addTasks(HT_SCHED::Scheduler& scheduler, std::size_t& currentTaskCount, std::size_t targetTaskCount)
{
    while (currentTaskCount < targetTaskCount)
    {
        tasks[currentTaskCount] = HT_TASK::Task(noOpTask, noOpTask, 0);

        if (!scheduler.schedule(tasks[currentTaskCount]))
        {
            Serial.println("Scheduler task queue is full");
            return;
        }
        ++currentTaskCount;
    }
}

///Runs and prints stats
void benchmarkScheduler(HT_SCHED::Scheduler& scheduler, std::size_t taskCount)
{
    Serial.println();
    Serial.printf("Benchmarking scheduler with %u tasks\n", static_cast<unsigned>(taskCount));

    ///Initialize
    scheduler.run();

    ///Warmup scheduler
    benchmark.warmup([&scheduler]()
    {
        scheduler.run();
    });
    ///Measure
    benchmark.measure([&scheduler]()
    {
        scheduler.run();
    });

    benchmark.print_results(Serial);
    benchmark.reset();
}

void setup()
{
    Serial.begin(115200);
    delay(1234);
    Serial.println();
    Serial.println("========================================");
    Serial.println("     HyTech Racing Scheduler Benchmark");
    Serial.println("========================================");
    Serial.printf("CPU frequency: %lu MHz\n", static_cast<unsigned long>(F_CPU_ACTUAL / 1000000));

    ///Initialize the scheduler
    HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();
    scheduler.setTimingFunction([]()
    {
        return static_cast<uint32_t>(micros());
    });

    benchmark.begin();
    std::size_t currentTaskCount = 0;

    for (const std::size_t taskCount : TASK_COUNTS)
    {
        addTasks(scheduler, currentTaskCount, taskCount);

        if (currentTaskCount != taskCount)
        {
            Serial.println("Scheduling failed");
            break;
        }
        benchmarkScheduler(scheduler, taskCount);
    }

    Serial.println();
    Serial.println("All scheduler benchmarks completed");
    Serial.flush();
}

void loop()
{
}


