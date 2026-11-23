#include "ht_sched.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>

namespace HT_SCHED
{
    void Scheduler::setTimingFunction(std::function<std::uint32_t()> microsFunction)
    {
        _microsFunction = std::move(microsFunction);
    }

    // Add a task to the scheduler's list
    // Give the task an ID
    // Return true upon success
    // Return false when the scheduling queue is full. (increase kMaxTasks if this happens)
    bool Scheduler::schedule(HT_TASK::Task& task)
    {
        if (_numTasks >= kMaxTasks)
        {
            return false;
        }

        // place the task at the back of the queue (last execution in a scheduling cycle)
        std::size_t i = _numTasks;
        _taskQueue[i] = &task;
        // start from the back and shift forward until priority ordering is satisfied
        // idle tasks are always lower priority than interval tasks
        // promote if either:
        // 1. tasks are the same type and priority of shifting task is higher
        // 2. shifting task is interval ask and compared task is idle
        while ((i > 0) &&
               (((task._taskInfo.priority < _taskQueue[i - 1]->_taskInfo.priority) &&
                 (task._taskInfo.isIntervalFunction == _taskQueue[i - 1]->_taskInfo.isIntervalFunction)) ||
                ((_taskQueue[i - 1]->_taskInfo.isIntervalFunction == false) &&
                 (task._taskInfo.isIntervalFunction == true))))
        {
            HT_TASK::Task* temp = _taskQueue[i];
            _taskQueue[i] = _taskQueue[i - 1];
            _taskQueue[i - 1] = temp;
            --i;
        }
        task._taskInfo.id = static_cast<unsigned int>(++_numTasks);
        return true;
    }

    void Scheduler::run()
    {
        if (!_microsFunction)
        {
            return;
        }

        // get the time
        const std::uint32_t nowMicros = _microsFunction();
        std::uint32_t cycleExecTimer = 0;
        // used to find the timing of the nearest interval function
        _timeOfNextExec = std::numeric_limits<std::uint32_t>::max();

        for (std::size_t i = 0; i < _numTasks; ++i)
        {
            HT_TASK::Task* task = _taskQueue[i];
            auto& info = task->_taskInfo;

            switch (info.state)
            {
            case HT_TASK::TaskState::UNINITIALIZED:
            {
                const HT_TASK::TaskResponse result = task->_setup(nowMicros, info);
                info.state = (result == HT_TASK::TaskResponse::EXIT)
                                 ? HT_TASK::TaskState::KILLED
                                 : HT_TASK::TaskState::RUNNING;

                info.nextExecutionMicros = nowMicros;
                info.lastExecutionMicros = nowMicros;
                ++info.executions;
                break;
            }
            case HT_TASK::TaskState::RUNNING:
            {
                if (info.isIntervalFunction)
                {
                    _timeOfNextExec = std::min(info.nextExecutionMicros, _timeOfNextExec);
                }

                const bool intervalDue =
                    info.isIntervalFunction &&
                    (static_cast<std::int32_t>(nowMicros - info.nextExecutionMicros) >= 0);

                const bool idleCanRun =
                    !info.isIntervalFunction &&
                    ((info.filteredExecutionDurationMicros + _microsFunction()) < _timeOfNextExec);

                if (intervalDue || idleCanRun)
                {
                    const std::uint32_t t0 = _microsFunction();
                    const HT_TASK::TaskResponse result = task->_loop(nowMicros, info);
                    const std::uint32_t t1 = _microsFunction();
                    const std::uint32_t dt = t1 - t0;

                    if (result == HT_TASK::TaskResponse::EXIT)
                    {
                        info.state = HT_TASK::TaskState::KILLED;
                    }

                    if (info.isIntervalFunction)
                    {
                        info.nextExecutionMicros += info.executionIntervalMicros;
                        _intervalExecTimer += dt;
                    }
                    else
                    {
                        _idleExecTimer += dt;
                    }

                    info.lastExecutionMicros = t1;
                    info.filteredExecutionDurationMicros = static_cast<std::uint32_t>((static_cast<float>(dt) * 0.5f) + (static_cast<float>(info.filteredExecutionDurationMicros) * 0.5f));
                    ++info.executions;
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
        const std::uint32_t endMicros = _microsFunction();
        _schedulerExecTimer += (endMicros - nowMicros) - cycleExecTimer;
    }

    HT_TASK::TaskResponse Scheduler::initSchedMon(const std::uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
    {
        (void)sysMicros;
        (void)taskInfo;
        _intervalExecTimer = 0;
        _idleExecTimer = 0;
        _schedulerExecTimer = 0;
        return HT_TASK::TaskResponse::YIELD;
    }

    HT_TASK::TaskResponse Scheduler::schedMon(const std::uint32_t& sysMicros, const HT_TASK::TaskInfo& taskInfo)
    {
        (void)sysMicros;
        (void)taskInfo;
        const std::uint32_t totalTime = _intervalExecTimer + _idleExecTimer + _schedulerExecTimer;
        if (totalTime > 0)
        {
            const float invTotal = 1.0f / static_cast<float>(totalTime);
            periodicUtilization = static_cast<float>(_intervalExecTimer) * invTotal;
            idleUtilization = static_cast<float>(_idleExecTimer) * invTotal;
            schedulerUtilization = static_cast<float>(_schedulerExecTimer) * invTotal;
        }
        else
        {
            periodicUtilization = 0.0f;
            idleUtilization = 0.0f;
            schedulerUtilization = 0.0f;
        }

        // reset counters
        _intervalExecTimer = 0;
        _idleExecTimer = 0;
        _schedulerExecTimer = 0;
        return HT_TASK::TaskResponse::YIELD;
    }

    const float& Scheduler::getPeriodicUtilization() const noexcept
    {
        return periodicUtilization;
    }

    const float& Scheduler::getIdleUtilization() const noexcept
    {
        return idleUtilization;
    }

    const float& Scheduler::getSchedulerUtilization() const noexcept
    {
        return schedulerUtilization;
    }
} // namespace HT_SCHED
