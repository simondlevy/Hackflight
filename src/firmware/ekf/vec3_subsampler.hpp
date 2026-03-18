/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2026 Simon D. Levy
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

#include <datatypes.hpp>

namespace hf {

    typedef struct {
        Vec3 sum;
        uint32_t count;
        float conversionFactor;
        Vec3 subSample;
    } Vec3SubSampler_t;

    static void vec3SubSamplerInit(Vec3SubSampler_t* subSampler, const
            float conversionFactor) { memset(subSampler, 0,
                sizeof(Vec3SubSampler_t));
            subSampler->conversionFactor = conversionFactor;
    }

    static void vec3SubSamplerAccumulate(Vec3SubSampler_t* subSampler,
            const Vec3* sample) {
        subSampler->sum.x += sample->x;
        subSampler->sum.y += sample->y;
        subSampler->sum.z += sample->z;

        subSampler->count++;
    }

    static Vec3* vec3SubSamplerFinalize(Vec3SubSampler_t* subSampler,
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
