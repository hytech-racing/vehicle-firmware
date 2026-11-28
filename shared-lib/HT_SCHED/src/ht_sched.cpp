#include "ht_sched.hpp"

#include <algorithm>

namespace HT_SCHED
{
    // Constructor
    Scheduler::Scheduler()
        : _num_tasks(0)
    {
    }

    void Scheduler::setTimingFunction(std::function<uint32_t()> micros_function)
    {
        _micros_function = micros_function;
    }

    // Add a task to the scheduler's list
    // Give the task an ID
    // Return true upon success
    // Return false when the scheduling queue is full. (increase HT_SCHED_MAX_TASKS if this happens)
    bool Scheduler::schedule(HT_TASK::Task& task)
    {
        if (!(_num_tasks < HT_SCHED_MAX_TASKS))
        {
            return false;
        }
        // place the task at the back of the queue (last execution in a scheduling cycle)
        _task_queue[_num_tasks] = &task;

        // start from the back (lowest priority) and shift forward until priority is correct
        // idle tasks are always lower priority than interval tasks
        // promote if either:
        // 1. tasks are the same type and priority of shifting task is higher
        // 2. shifting task is interval ask and compared task is idle
        int i = _num_tasks;
        while ((i > 0)
               && (
                   // check tasks are the same type and priority of shifting task is higher
                   ((task.task_info.priority < _task_queue[i - 1]->task_info.priority)
                    && (task.task_info.is_interval_function == _task_queue[i - 1]->task_info.is_interval_function))
                   // check shifting task is interval ask and compared task is idle
                   || ((_task_queue[i - 1]->task_info.is_interval_function == false) && (task.task_info.is_interval_function == true))))
        {
            HT_TASK::Task* temp = _task_queue[i];
            _task_queue[i] = _task_queue[i - 1];
            _task_queue[i - 1] = temp;
        }

        task.task_info.id = ++_num_tasks;

        return true;
    }

    void Scheduler::run()
    {
        // get the time
        uint32_t now_micros = (uint32_t)_micros_function();
        uint32_t cycle_exec_timer = 0;
        // used to find the timing of the nearest interval function
        _time_of_next_exec = ULONG_MAX;

        for (int i = 0; i < _num_tasks; i++)
        {
            HT_TASK::Task* task = _task_queue[i];

            switch (task->task_info.state)
            {
            case HT_TASK::TaskState::UNINITIALIZED:
            {
                // enable the task
                // run its setup function
                // check if the task wants to exit
                HT_TASK::TaskResponse result = task->setup(now_micros, task->task_info);
                if (result == HT_TASK::TaskResponse::EXIT)
                    task->task_info.state = HT_TASK::TaskState::KILLED;
                else
                    task->task_info.state = HT_TASK::TaskState::RUNNING;

                task->task_info.next_execution_micros = now_micros;
                task->task_info.last_execution_micros = now_micros;
                task->task_info.executions++;
                break;
            }
            case HT_TASK::TaskState::RUNNING:
            {
                // find the time the next interval function is scheduled to run
                if (task->task_info.is_interval_function)
                    _time_of_next_exec = std::min(task->task_info.next_execution_micros, _time_of_next_exec);

                // check if it's time to execute
                if (
                    // for interval functions, see if next_execution_micros has passed
                    ((task->task_info.is_interval_function == true)
                     && (task->task_info.next_execution_micros - now_micros >= task->task_info.execution_interval_micros))
                    // for idle functions, see if time might conflict with interval function
                    || ((task->task_info.is_interval_function == false)
                        && (task->task_info.filtered_execution_duration_micros + _micros_function() < _time_of_next_exec)))
                {
                    // run its loop function
                    uint32_t dt = (uint32_t)_micros_function();
                    HT_TASK::TaskResponse result = task->loop(now_micros, task->task_info);
                    dt = _micros_function() - dt;
                    if (result == HT_TASK::EXIT)
                        task->task_info.state = HT_TASK::TaskState::KILLED;

                    task->task_info.next_execution_micros += task->task_info.execution_interval_micros;
                    task->task_info.last_execution_micros = _micros_function();
                    task->task_info.filtered_execution_duration_micros = (dt * 0.5) + (task->task_info.filtered_execution_duration_micros * 0.5);
                    task->task_info.executions++;

                    // track execution time
                    if (task->task_info.is_interval_function)
                        _interval_exec_timer += dt;
                    else
                        _idle_exec_timer += dt;
                    cycle_exec_timer += dt;
                }
                break;
            }
            case HT_TASK::TaskState::KILLED:
            default:
                break;
            }
        }

        // track time in the scheduler
        _scheduler_exec_timer += _micros_function() - now_micros - cycle_exec_timer;
    }

    HT_TASK::TaskResponse Scheduler::initSchedMon(const uint32_t& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        _interval_exec_timer = 0;
        _idle_exec_timer = 0;
        _scheduler_exec_timer = 0;

        return HT_TASK::TaskResponse::YIELD;
    }

    HT_TASK::TaskResponse Scheduler::schedMon(const uint32_t& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        uint32_t total_time = _interval_exec_timer + _idle_exec_timer + _scheduler_exec_timer;
        _periodic_utilization = (float)_interval_exec_timer / total_time;
        _idle_utilization = (float)_idle_exec_timer / total_time;
        _scheduler_utilization = (float)_scheduler_exec_timer / total_time;

        // reset counters
        _interval_exec_timer = 0;
        _idle_exec_timer = 0;
        _scheduler_exec_timer = 0;

        // std::cout << "p_util: " << _periodic_utilization << ", i_util: " << _idle_utilization << ", s_util:" << _scheduler_utilization << "\n";

        return HT_TASK::TaskResponse::YIELD;
    }

    const float& Scheduler::getPeriodicUtilization()
    {
        return _periodic_utilization;
    }

    const float& Scheduler::getIdleUtilization()
    {
        return _idle_utilization;
    }

    const float& Scheduler::getSchedulerUtilization()
    {
        return _scheduler_utilization;
    }
} // namespace HT_SCHED
