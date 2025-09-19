#pragma once

#include <tasks/debug.hpp>

class FooTask {

    public:

        void begin(DebugTask * debugTask)
        {
            _debugTask = debugTask;

            _task.init(runFooTask, "foo", this, 3);
        }

    private:

        FreeRtosTask _task;

        DebugTask * _debugTask;

        static void runFooTask(void *obj)
        {
            ((FooTask *)obj)->run();
        }

        void run(void)
        {
            while (true) {

                static uint32_t count;
                DebugTask::setMessage(_debugTask, "FooTask: %d", +count++);
                vTaskDelay(1);

            }
        }
};
