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

#include <Arduino.h>

#include <datatypes.h>

class OpticalFlow {

    public:

        typedef struct {
            uint32_t timestamp;
            union {
                struct {
                    float dpixelx;  // Accumulated pixel count x
                    float dpixely;  // Accumulated pixel count y
                };
                float dpixel[2];  // Accumulated pixel count
            };
            float stdDevX;      // Measurement standard deviation
            float stdDevY;      // Measurement standard deviation
            float dt;           // Time during which pixels were accumulated
        } measurement_t;


        void init()
        {
            device_init();
        }

        bool read(measurement_t & flowData)
        {
            int16_t deltaX = 0;
            int16_t deltaY = 0;
            bool gotMotion = false;

            device_read(deltaX, deltaY, gotMotion);

            // Flip motion information to comply with sensor mounting
            // (might need to be changed if mounted differently)
            int16_t accpx = -deltaY;
            int16_t accpy = -deltaX;

            // Outlier removal
            if (abs(accpx) < OUTLIER_LIMIT && abs(accpy) < OUTLIER_LIMIT) {

                static uint32_t _lastTime;

                // Form flow measurement struct and push into the EKF
                flowData.stdDevX = FLOW_STD_FIXED;
                flowData.stdDevY = FLOW_STD_FIXED;
                flowData.dt = (float)(micros()-_lastTime)/1000000.0f;
                _lastTime = micros();

                // Use raw measurements
                flowData.dpixelx = (float)accpx;
                flowData.dpixely = (float)accpy;

                // Push measurements into the estimator if flow is not disabled
                //    and the PMW flow sensor indicates motion detection
                if (!USE_FLOW_DISABLED && gotMotion) {
                    return true;
                }
            }

            return false;
        }        

    private:

        static const int16_t OUTLIER_LIMIT = 100;

        // Disables pushing the flow measurement in the EKF
        static const auto USE_FLOW_DISABLED = false;

        // Set standard deviation flow
        static constexpr float FLOW_STD_FIXED = 2.0;

        bool device_init();

        void device_read(int16_t & dx, int16_t & dy, bool &gotMotion);
};
