#ifndef __HT_SCHED__
#define __HT_SCHED__

#include "ht_task.hpp"

#include <climits>
#include <functional>

#define HT_SCHED_MAX_TASKS 32

namespace HT_SCHED
{
    class Scheduler
    {
      public:
        // Singleton patterning
        static Scheduler& getInstance()
        {
            static Scheduler instance;
            return instance;
        }

        void setTimingFunction(std::function<uint32_t()>);
        bool schedule(HT_TASK::Task& task);
        void run();

        // init function for SchedMon task
        HT_TASK::TaskResponse initSchedMon(const uint32_t& sys_micros, const HT_TASK::TaskInfo& task_info);
        // loop for SchedMon task
        HT_TASK::TaskResponse schedMon(const uint32_t& sys_micros, const HT_TASK::TaskInfo& task_info);
        const float& getPeriodicUtilization();
        const float& getIdleUtilization();
        const float& getSchedulerUtilization();

      private:
        Scheduler();

        HT_TASK::Task* _task_queue[HT_SCHED_MAX_TASKS]; // ordered list of task object pointers
        uint32_t _num_tasks;
        std::function<uint32_t()> _micros_function;
        uint32_t _time_of_next_exec;    // time at which the next scheduled function must trigger
        uint32_t _interval_exec_timer;  // accumulation of time spent executing scheduled tasks
        uint32_t _idle_exec_timer;      // accumulation of time spent executing idle tasks
        uint32_t _scheduler_exec_timer; // accumulation of time spent in the scheduler

        // variables for SchedMon
        float _periodic_utilization;  // CPU utilization of periodic tasks
        float _idle_utilization;      // CPU utilization of idle tasks
        float _scheduler_utilization; // CPU utilization of the scheduler
    };
} // namespace HT_SCHED


#endif // __HT_SCHED__
