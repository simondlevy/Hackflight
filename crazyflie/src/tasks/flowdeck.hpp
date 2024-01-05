#pragma once

#include <stdlib.h>

#include <free_rtos.h>
#include <task.h>


// Arduino library
#include <pmw3901.hpp>

#include <datatypes.h>

#include <tasks/estimator.hpp>

#include <crossplatform.h>
#include <pinmap.h>
#include <system.h>

void flowdeckInit(void);

class FlowDeckTask {

    public:

        // Shared with params
        bool didInit;

        void init(EstimatorTask * estimatorTask)
        {
            if (didInit) {
                return;
            }

            _estimatorTask = estimatorTask;


            if (_pmw3901.begin(PIN_FLOWDECK_CS)) {

                xTaskCreate(flowdeckTask, "FLOW", STACKSIZE, this, 3, NULL);

                didInit = true;
            }
        }

    private:

        static const auto STACKSIZE = 2 * configMINIMAL_STACK_SIZE;

        static const int16_t OUTLIER_LIMIT = 100;

        // Disables pushing the flow measurement in the EKF
        static const auto USE_FLOW_DISABLED = false;

        // Set standard deviation flow
        static constexpr float FLOW_STD_FIXED = 2.0;

        static void flowdeckTask(void *param)
        {
            ((FlowDeckTask *)param)->run();
        }

        PMW3901 _pmw3901;

        EstimatorTask * _estimatorTask;

        void run(void)
        {
            systemWaitStart();

            auto lastTime  = micros();

            while (true) {

                vTaskDelay(10);

                int16_t deltaX = 0;
                int16_t deltaY = 0;
                bool gotMotion = false;

                _pmw3901.readMotion(deltaX, deltaY, gotMotion);

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
                        _estimatorTask->enqueueFlow(&flowData, hal_isInInterrupt());
                    }
                }
            }        
        }
};
