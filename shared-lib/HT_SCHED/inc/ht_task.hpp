#ifndef __HT_TASK__
#define __HT_TASK__

#include <functional>
#include <stdint.h>

namespace HT_TASK
{
    enum TaskState
    {
        UNINITIALIZED = 0,
        RUNNING = 1,
        KILLED = 2,
        NUM_STATES
    };

    enum TaskResponse
    {
        YIELD = 0, // If the task should run again (not permanently end)
        EXIT = 1,  // If the task should permanently end
    };

    class TaskInfo
    {
      public:
        TaskState state;
        uint32_t execution_interval_micros;
        uint32_t last_execution_micros;
        uint32_t next_execution_micros;
        uint32_t filtered_execution_duration_micros;
        uint32_t executions;
        unsigned int id;
        // use priority to enforce ordering when multiple tasks execute on the same scheduling cycle
        unsigned int priority; // 0 is highest priority.
        bool is_interval_function;

        TaskInfo()
            : state(TaskState::UNINITIALIZED),
              execution_interval_micros(1),
              last_execution_micros(0),
              next_execution_micros(0),
              filtered_execution_duration_micros(0),
              executions(0),
              id(0),
              priority(0),
              is_interval_function(false)
        {
        }
    };

    // Use this to declare a task without a setup or loop
    inline TaskResponse dummyFunction(const uint32_t& time_micros, const TaskInfo& task_info)
    {
        return TaskResponse::YIELD;
    }

    class Task
    {
      public:
        TaskInfo task_info;
        std::function<TaskResponse(const uint32_t&, const TaskInfo&)> setup;
        std::function<TaskResponse(const uint32_t&, const TaskInfo&)> loop;

        // Interval task constructor
        Task(
            std::function<TaskResponse(const uint32_t&, const TaskInfo&)> setup,
            std::function<TaskResponse(const uint32_t&, const TaskInfo&)> loop,
            int priority,
            uint32_t execution_interval_micros)
            : task_info(TaskInfo()), setup(setup), loop(loop)
        {
            task_info.execution_interval_micros = execution_interval_micros;
            task_info.is_interval_function = true;
            task_info.priority = priority;
        };

        // Idle task constructor with priority
        Task(
            std::function<TaskResponse(const uint32_t&, const TaskInfo&)> setup,
            std::function<TaskResponse(const uint32_t&, const TaskInfo&)> loop,
            int priority)
            : task_info(TaskInfo()), setup(setup), loop(loop)
        {
            task_info.is_interval_function = false;
            task_info.priority = priority;
        };

        // default constructor
        Task()
            : Task(dummyFunction, dummyFunction, 0) {};
    };
} // namespace HT_TASK


#endif // __HT_TASK__
