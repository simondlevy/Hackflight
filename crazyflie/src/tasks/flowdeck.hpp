/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdlib.h>

// Arduino library
#include <pmw3901.hpp>

#include <datatypes.h>

#include <task.hpp>
#include <tasks/estimator.hpp>

#include <crossplatform.h>

void flowdeckInit(void);

class FlowDeckTask : public FreeRTOSTask {

    public:

        void begin(const uint8_t csPin, EstimatorTask * estimatorTask)
        {
            if (didInit) {
                return;
            }

            _estimatorTask = estimatorTask;


            if (_pmw3901.begin(csPin)) {

                FreeRTOSTask::begin(runFlowdeckTask, "flow", this, 3);

                didInit = true;
            }
        }

    private:

        static const int16_t OUTLIER_LIMIT = 100;

        // Disables pushing the flow measurement in the EKF
        static const auto USE_FLOW_DISABLED = false;

        // Set standard deviation flow
        static constexpr float FLOW_STD_FIXED = 2.0;

        static void runFlowdeckTask(void *obj)
        {
            ((FlowDeckTask *)obj)->run();
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
