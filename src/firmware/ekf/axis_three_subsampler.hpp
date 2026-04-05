/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <firmware/imu/sensor.hpp>


namespace hf {

    class ThreeAxisSubSampler {

        public:

            IMU::ThreeAxis sum;
            uint32_t count;
            float conversionFactor;
            IMU::ThreeAxis subSample;

            ThreeAxisSubSampler() = default;

            ThreeAxisSubSampler& operator=(const ThreeAxisSubSampler& other) = default;
 
            ThreeAxisSubSampler
                (IMU::ThreeAxis sum,
                 uint32_t count,
                 float conversionFactor,
                 IMU::ThreeAxis subSample)
                :
                    sum(sum),
                    count(count),
                    conversionFactor(conversionFactor),
                    subSample(subSample) {}
    };

    typedef struct {
        IMU::ThreeAxis sum;
        uint32_t count;
        float conversionFactor;
        IMU::ThreeAxis subSample;
    } ThreeAxisSubSampler_t;

    static void axis3fSubSamplerInit(ThreeAxisSubSampler_t* subSampler, const
            float conversionFactor)
    {
        memset(subSampler, 0, sizeof(ThreeAxisSubSampler_t));
        subSampler->conversionFactor = conversionFactor;
    }

    static void axis3fSubSamplerAccumulate(ThreeAxisSubSampler_t* subSampler,
            const IMU::ThreeAxis* sample)
    {
        subSampler->sum.x += sample->x;
        subSampler->sum.y += sample->y;
        subSampler->sum.z += sample->z;

        subSampler->count++;
    }

    static IMU::ThreeAxis* axis3fSubSamplerFinalize(ThreeAxisSubSampler_t* subSampler,
            const char * label) 
    {
        if (subSampler->count > 0) {

            subSampler->subSample.x = 
                subSampler->sum.x * subSampler->conversionFactor / subSampler->count;
            subSampler->subSample.y = 
                subSampler->sum.y * subSampler->conversionFactor / subSampler->count;
            subSampler->subSample.z = 
                subSampler->sum.z * subSampler->conversionFactor / subSampler->count;

            // Reset
            subSampler->count = 0;
            subSampler->sum.x = 0;
            subSampler->sum.y = 0;
            subSampler->sum.z = 0;
        }

        return &subSampler->subSample;
    }

}
