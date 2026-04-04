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

#include <hackflight.h>
#include <firmware/opticalflow/sensor.hpp>

namespace hf {

    class OpticalFlowFilter {

        private:

            static constexpr int16_t OUTLIER_LIMIT = 100;

            static constexpr float FLOW_STD_FIXED = 2.0;

        public:

            OpticalFlowFilter() = default;

            OpticalFlowFilter& operator=(const OpticalFlowFilter& other) = default;

            OpticalFlowFilter(
                    const bool gotMotion,
                    const uint32_t usec_prev,
                    const float dpixelx,  
                    const float dpixely, 
                    const float stdDevX,
                    const float stdDevY,
                    const float dt)
                : 
                    _gotMotion(gotMotion),
                    _usec_prev(usec_prev),
                    _dpixelx(dpixelx),  
                    _dpixely(dpixely), 
                    _stdDevX(stdDevX),
                    _stdDevY(stdDevY),
                    _dt(dt) {}

            static auto step(
                    const OpticalFlowFilter & filter,
                    const uint32_t usec_curr,
                    const OpticalFlowSensor::RawData &
                    rawdata) -> OpticalFlowFilter
            {
                // Flip motion information to comply with sensor mounting
                // (might need to be changed if mounted differently)
                const int16_t accpx = -rawdata.y;
                const int16_t accpy = -rawdata.x;

                return inlimit(accpx) && inlimit(accpy) ?

                    OpticalFlowFilter(
                            true,           // got motion
                            usec_curr,      // usec_prev
                            (float)accpx,   // dpixelx
                            (float)accpy,   // dpixely
                            FLOW_STD_FIXED, // stdDevX
                            FLOW_STD_FIXED, // stdDevY
                            (float)(usec_curr - filter._usec_prev) / 1e6 // dt
                            ) :
                    filter;
            }        

        private:

            bool _gotMotion;
            uint32_t _timestamp_usec;
            uint32_t _usec_prev;
            float _dpixelx;  // Accumulated pixel count x
            float _dpixely;  // Accumulated pixel count y
            float _stdDevX;  // Measurement standard deviation
            float _stdDevY;  // Measurement standard deviation
            float _dt;       // Time during which pixels were accumulated


            static auto inlimit(const int16_t accval) -> bool
            {
                return abs(accval) < OUTLIER_LIMIT;
            }

    };
}
