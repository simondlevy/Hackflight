#pragma once

#include <pmw3901.hpp>

#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>

class OpticalFlowTask {

    public:

        void begin(EstimatorTask * estimatorTask, const uint8_t csPin, DebugTask * debugTask)
        {
            if (_task.didInit()){
                return;
            }

            _estimatorTask = estimatorTask;
            _debugTask = debugTask;

            if (_pmw3901.begin(csPin)) {
                _task.init(runOpticalFlowTask, "flow", this, 3);
            }
            else {
                debugTask->setMessage("PMW3901 initialization failed.");
            }
        }

    private:

        static const int16_t OUTLIER_LIMIT = 100;

        // Disables pushing the flow measurement in the EKF
        static const auto USE_FLOW_DISABLED = false;

        // Set standard deviation flow
        static constexpr float FLOW_STD_FIXED = 2.0;

        static void runOpticalFlowTask(void * obj)
        {
            ((OpticalFlowTask *)obj)->run();
        }

        PMW3901 _pmw3901;

        FreeRtosTask _task;

        float _expCoeff;

        EstimatorTask * _estimatorTask;

        DebugTask * _debugTask;

        void run(void)
        {
            auto lastTime  = micros();

            while (true) {

                vTaskDelay(10);

                int16_t deltaX = 0;
                int16_t deltaY = 0;
                bool gotMotion = false;

                _pmw3901.readMotion(deltaX, deltaY, gotMotion);

                _debugTask->setMessage("gotMotion=%s dx=%+03d dy=%+03d",
                        gotMotion ? "yes" : "no ", deltaX, deltaY);

                // Flip motion information to comply with sensor mounting
                // (might need to be changed if mounted differently)
                int16_t accpx = -deltaY;
                int16_t accpy = -deltaX;

                // Outlier removal
                if (abs(accpx) < OUTLIER_LIMIT && abs(accpy) < OUTLIER_LIMIT) {

                    // Form flow measurement struct and push into the EKF
                    flowMeasurement_t flowData;
                    flowData.stdDevX = FLOW_STD_FIXED;
                    flowData.stdDevY = FLOW_STD_FIXED;
                    flowData.dt = (float)(micros()-lastTime)/1000000.0f;
                    // we do want to update dt every measurement and not only in the
                    // ones with detected motion, as we work with instantaneous gyro
                    // and velocity values in the update function
                    // (meaning assuming the current measurements over all of dt)
                    lastTime = micros();

                    // Use raw measurements
                    flowData.dpixelx = (float)accpx;
                    flowData.dpixely = (float)accpy;

                    // Push measurements into the estimator if flow is not disabled
                    //    and the PMW flow sensor indicates motion detection
                    if (!USE_FLOW_DISABLED && gotMotion) {
                        _estimatorTask->enqueueFlow(&flowData, xPortIsInsideInterrupt());
                    }
                }
             }
        }
};
