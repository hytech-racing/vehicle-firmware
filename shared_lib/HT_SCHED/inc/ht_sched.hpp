#ifndef __HT_SCHED__
#define __HT_SCHED__

#include <functional>
#include "ht_task.hpp"
#include <climits>

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
        HT_TASK::TaskResponse initSchedMon(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo);
        // loop for SchedMon task
        HT_TASK::TaskResponse schedMon(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo);
        const float& getPeriodicUtilization();
        const float& getIdleUtilization();
        const float& getSchedulerUtilization();
        
        private:
        
        Scheduler();

        HT_TASK::Task*                  _taskQueue[HT_SCHED_MAX_TASKS]; // ordered list of task object pointers
        uint32_t                        _numTasks;
        std::function<uint32_t()>       _microsFunction;
        uint32_t                        _timeOfNextExec;        // time at which the next scheduled function must trigger
        uint32_t                        _intervalExecTimer;     // accumulation of time spent executing scheduled tasks
        uint32_t                        _idleExecTimer;         // accumulation of time spent executing idle tasks
        uint32_t                        _schedulerExecTimer;    // accumulation of time spent in the scheduler

        // variables for SchedMon
        float periodicUtilization;  // CPU utilization of periodic tasks
        float idleUtilization;      // CPU utilization of idle tasks
        float schedulerUtilization; // CPU utilization of the scheduler
    };
} // namespace HT_SCHED


#endif // __HT_SCHED__
