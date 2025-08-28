#pragma once

#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>

class FlowDeckTask {

    public:

        void begin(EstimatorTask * estimatorTask, const uint8_t csPin, DebugTask * debugTask)
        {
            if (_task.didInit()){
                return;
            }

            _estimatorTask = estimatorTask;
            _debugTask = debugTask;

            _task.init(runFlowDeckTask, "flow", this, 3);
        }

    private:

        static constexpr float FREQ_HZ = 40;

        static void runFlowDeckTask(void * obj)
        {
            ((FlowDeckTask *)obj)->run();
        }

        FreeRtosTask _task;

        float _expCoeff;

        EstimatorTask * _estimatorTask;

        DebugTask * _debugTask;

        void run(void)
        {
            TickType_t lastWakeTime;

            lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, M2T(1000/FREQ_HZ));

                _debugTask->setMessage("%lu", micros());

            }
        }
};
