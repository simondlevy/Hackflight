/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#include <pmw3901.hpp>

#include <debugger.hpp>
#include <ekf.hpp>

class OpticalFlow {

    public:

        void init(SPIClass * spi, const uint8_t cs_pin)
        {

            if (!_pmw3901.begin(cs_pin, *spi)) {
                Debugger::error("OpticalFlow");
            }
        }

        void step(EKF * ekf)
        {
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

                static uint32_t _msecPrev;

                // Form flow measurement struct and push into the EKF
                flowMeasurement_t flowData;
                flowData.stdDevX = FLOW_STD_FIXED;
                flowData.stdDevY = FLOW_STD_FIXED;
                flowData.dt = (float)(micros()-_msecPrev)/1000000.0f;

                // We do want to update dt every measurement and not only
                // in the ones with detected motion, as we work with
                // instantaneous gyro and velocity values in the update
                // function (meaning assuming the current measurements over
                // all of dt)
                _msecPrev = micros();

                // Use raw measurements
                flowData.dpixelx = (float)accpx;
                flowData.dpixely = (float)accpy;

                // Push measurements into the ekf if flow is not disabled
                //    and the PMW flow sensor indicates motion detection
                if (gotMotion) {
                    ekf->enqueueFlow(&flowData);
                }
            }
        }

    private:

        static constexpr float TASK_FREQ = 100;

        static const int16_t OUTLIER_LIMIT = 100;

        // Set standard deviation flow
        static constexpr float FLOW_STD_FIXED = 2.0;

        PMW3901 _pmw3901;
};
