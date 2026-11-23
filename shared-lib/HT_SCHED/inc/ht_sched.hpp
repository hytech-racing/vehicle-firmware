#ifndef __HT_SCHED__
#define __HT_SCHED__

#include "ht_task.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <limits>

namespace HT_SCHED
{
    inline constexpr std::size_t kMaxTasks = 32;
    class Scheduler
    {
      public:
        // Singleton patterning
        [[nodiscard]] static Scheduler& getInstance() noexcept
        {
            static Scheduler instance;
            return instance;
        }

        Scheduler(const Scheduler&) = delete;
        Scheduler& operator=(const Scheduler&) = delete;
        Scheduler(Scheduler&&) = delete;
        Scheduler& operator=(Scheduler&&) = delete;

        void setTimingFunction(std::function<std::uint32_t()> microsFunction);
        [[nodiscard]] bool schedule(HT_TASK::Task& task);
        void run();

        // init function for SchedMon task
        HT_TASK::TaskResponse initSchedMon(const std::uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo);
        // loop for SchedMon task
        HT_TASK::TaskResponse schedMon(const std::uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo);

        [[nodiscard]] const float& getPeriodicUtilization() const noexcept;
        [[nodiscard]] const float& getIdleUtilization() const noexcept;
        [[nodiscard]] const float& getSchedulerUtilization() const noexcept;

      private:
        Scheduler() noexcept = default;
        std::array<HT_TASK::Task*, kMaxTasks> _taskQueue{}; // ordered list of task object pointers
        std::size_t _numTasks{0};
        std::function<std::uint32_t()> _microsFunction{};
        std::uint32_t _timeOfNextExec{std::numeric_limits<std::uint32_t>::max()}; // time at which the next scheduled function must trigger
        std::uint32_t _intervalExecTimer{0};  // accumulation of time spent executing scheduled tasks
        std::uint32_t _idleExecTimer{0};      // accumulation of time spent executing idle tasks
        std::uint32_t _schedulerExecTimer{0}; // accumulation of time spent in the scheduler

        // variables for SchedMon
        float periodicUtilization{0.0f};  // CPU utilization of periodic tasks
        float idleUtilization{0.0f};      // CPU utilization of idle tasks
        float schedulerUtilization{0.0f}; // CPU utilization of the scheduler
    };
} // namespace HT_SCHED

#endif // __HT_SCHED__
