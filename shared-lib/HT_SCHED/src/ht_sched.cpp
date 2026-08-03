#include "ht_sched.hpp"
#include <algorithm>

namespace HT_SCHED
{
    // Constructor
    Scheduler::Scheduler() :
        _numTasks(0)
    {
        
    }

    void Scheduler::setTimingFunction(std::function<uint32_t()> microsFunction)
    {
        _microsFunction = microsFunction;
    }

    // Add a task to the scheduler's list
    // Give the task an ID
    // Return true upon success
    // Return false when the scheduling queue is full. (increase HT_SCHED_MAX_TASKS if this happens)
    bool Scheduler::schedule(HT_TASK::Task& task)
    {
        if (!(_numTasks < HT_SCHED_MAX_TASKS))
        {
            return false;
        }
        // place the task at the back of the queue (last execution in a scheduling cycle)
        _taskQueue[_numTasks] = &task;
        
        // start from the back (lowest priority) and shift forward until priority is correct
        // idle tasks are always lower priority than interval tasks
        // promote if either:
        // 1. tasks are the same type and priority of shifting task is higher
        // 2. shifting task is interval ask and compared task is idle
        int i = _numTasks;
        while (
            (i > 0)
            && (
                // check tasks are the same type and priority of shifting task is higher
                (
                    (task._taskInfo.priority < _taskQueue[i-1]->_taskInfo.priority) 
                    && (task._taskInfo.isIntervalFunction == _taskQueue[i-1]->_taskInfo.isIntervalFunction)
                )
                // check shifting task is interval ask and compared task is idle
                || (
                    (_taskQueue[i-1]->_taskInfo.isIntervalFunction == false)
                    && (task._taskInfo.isIntervalFunction == true)
                )
            )
        )
        {
            HT_TASK::Task* temp = _taskQueue[i];
            _taskQueue[i] = _taskQueue[i-1];
            _taskQueue[i-1] = temp;
        }

        task._taskInfo.id = ++_numTasks;

        return true;

    }

    void Scheduler::run()
    {
        // get the time
        uint32_t nowMicros = (uint32_t) _microsFunction();
        uint32_t cycleExecTimer = 0;
        // used to find the timing of the nearest interval function
        _timeOfNextExec = ULONG_MAX;

        for (int i = 0; i < _numTasks; i++)
        {
            HT_TASK::Task* task = _taskQueue[i];

            switch (task->_taskInfo.state)
            {
                case HT_TASK::TaskState::UNINITIALIZED:
                    {
                        // enable the task
                        // run its setup function
                        // check if the task wants to exit
                        HT_TASK::TaskResponse result = task->_setup(nowMicros, task->_taskInfo);
                        if (result == HT_TASK::TaskResponse::EXIT)
                            task->_taskInfo.state = HT_TASK::TaskState::KILLED;
                        else
                            task->_taskInfo.state = HT_TASK::TaskState::RUNNING;

                        task->_taskInfo.nextExecutionMicros = nowMicros;
                        task->_taskInfo.lastExecutionMicros = nowMicros;
                        task->_taskInfo.executions++;
                        break;
                    }
                case HT_TASK::TaskState::RUNNING:
                    {
                        // find the time the next interval function is scheduled to run
                        if (task->_taskInfo.isIntervalFunction)
                            _timeOfNextExec = std::min(task->_taskInfo.nextExecutionMicros, _timeOfNextExec);

                        // check if it's time to execute
                        if (
                            // for interval functions, see if nextExecutionMicros has passed
                            ((task->_taskInfo.isIntervalFunction == true) && (task->_taskInfo.nextExecutionMicros - nowMicros >= task->_taskInfo.executionIntervalMicros))
                            // for idle functions, see if time might conflict with interval function
                            || ((task->_taskInfo.isIntervalFunction == false) && (task->_taskInfo.filteredExecutionDurationMicros + _microsFunction() < _timeOfNextExec))
                        )
                        {
                            // run its loop function
                            uint32_t dt = (uint32_t) _microsFunction();
                            HT_TASK::TaskResponse result = task->_loop(nowMicros, task->_taskInfo);
                            dt = _microsFunction() - dt;
                            if (result == HT_TASK::EXIT)
                                task->_taskInfo.state = HT_TASK::TaskState::KILLED;

                            task->_taskInfo.nextExecutionMicros += task->_taskInfo.executionIntervalMicros;
                            task->_taskInfo.lastExecutionMicros = _microsFunction();
                            task->_taskInfo.filteredExecutionDurationMicros = (dt * 0.5) + (task->_taskInfo.filteredExecutionDurationMicros * 0.5);
                            task->_taskInfo.executions++;

                            // track execution time
                            if (task->_taskInfo.isIntervalFunction)
                                _intervalExecTimer += dt;
                            else
                                _idleExecTimer += dt;
                            cycleExecTimer += dt;
                        }
                        break;
                    }
                case HT_TASK::TaskState::KILLED:
                default:
                    break;
            }
        }

        // track time in the scheduler
        _schedulerExecTimer += _microsFunction() - nowMicros - cycleExecTimer;
    }

    HT_TASK::TaskResponse Scheduler::initSchedMon(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
    {
        _intervalExecTimer = 0;
        _idleExecTimer = 0;
        _schedulerExecTimer = 0;

        return HT_TASK::TaskResponse::YIELD;
    }
    
    HT_TASK::TaskResponse Scheduler::schedMon(const uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
    {
        uint32_t totalTime = _intervalExecTimer + _idleExecTimer + _schedulerExecTimer;
        periodicUtilization     = (float) _intervalExecTimer    / totalTime;
        idleUtilization         = (float) _idleExecTimer        / totalTime;
        schedulerUtilization    = (float)_schedulerExecTimer    / totalTime;

        // reset counters
        _intervalExecTimer = 0;
        _idleExecTimer = 0;
        _schedulerExecTimer = 0;

        // std::cout << "p_util: " << periodicUtilization << ", i_util: " << idleUtilization << ", s_util:" << schedulerUtilization << "\n";

        return HT_TASK::TaskResponse::YIELD;
    }

    const float& Scheduler::getPeriodicUtilization()
    {
        return periodicUtilization;
    }

    const float& Scheduler::getIdleUtilization()
    {
        return idleUtilization;
    }

    const float& Scheduler::getSchedulerUtilization()
    {
        return schedulerUtilization;
    }
}
