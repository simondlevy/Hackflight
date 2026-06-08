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

            bool got_motion;
            uint32_t timestamp_usec;
            uint32_t usec_prev;
            float dpixelx;  // Accumulated pixel count x
            float dpixely;  // Accumulated pixel count y
            float std_dev_x;  // Measurement standard deviation
            float std_dev_y;  // Measurement standard deviation
            float dt;       // Time during which pixels were accumulated

            OpticalFlowFilter() = default;

            OpticalFlowFilter& operator=(const OpticalFlowFilter& other) = default;

            OpticalFlowFilter(
                    const bool got_motion,
                    const uint32_t usec_prev,
                    const float dpixelx,  
                    const float dpixely, 
                    const float std_dev_x,
                    const float std_dev_y,
                    const float dt)
                : 
                    got_motion(got_motion),
                    usec_prev(usec_prev),
                    dpixelx(dpixelx),  
                    dpixely(dpixely), 
                    std_dev_x(std_dev_x),
                    std_dev_y(std_dev_y),
                    dt(dt) {}

            static auto Update(
                    const OpticalFlowFilter & filter,
                    const uint32_t usec_curr,
                    const OpticalFlowSensor::RawData & rawdata
                    ) -> OpticalFlowFilter
            {
                // Flip motion information to comply with sensor mounting
                // (might need to be changed if mounted differently)
                const int16_t accpx = rawdata.y;
                const int16_t accpy = rawdata.x;

                return IsInLimit(accpx) && IsInLimit(accpy) ?

                    OpticalFlowFilter(
                            true,           // got motion
                            usec_curr,      // usec_prev
                            (float)accpx,   // dpixelx
                            (float)accpy,   // dpixely
                            FLOW_STD_FIXED, // std_dev_x
                            FLOW_STD_FIXED, // std_dev_y
                            (float)(usec_curr - filter.usec_prev) / 1e6 // dt
                            ) :
                    filter;
            }        

        private:

            static auto IsInLimit(const int16_t accval) -> bool
            {
                return abs(accval) < OUTLIER_LIMIT;
            }

    };
}
