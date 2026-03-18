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

    class Vec3SubSampler {

        public:

            float conversionFactor;
            Vec3 sum;
            uint32_t count;
            Vec3 subSample;

            Vec3SubSampler() = default;

            Vec3SubSampler(
                    const float conversionFactor,
                    const Vec3 & sum = {},
                    const uint32_t count = 0,
                    const Vec3 & subSample = {})
                : 
                    conversionFactor(conversionFactor),
                    sum(sum),
                    count(count),
                    subSample(subSample) {}

            static void reset(Vec3SubSampler & ss)
            {
                ss.sum = {};
                ss.count = 0;
            }

            static void init(Vec3SubSampler & ss,
                    const float conversionFactor) 
            {
                reset(ss);
                ss.conversionFactor = conversionFactor;
            }

            static void accumulate(Vec3SubSampler & ss,
                    const Vec3 & sample)
            {
                ss.sum = ss.sum + sample;
                ss.count++;
            }

            static void finalize(Vec3SubSampler & ss)
            {
                if (ss.count > 0) {
                    ss.subSample = ss.sum * ss.conversionFactor / ss.count;
                    reset(ss);
                }
            }
    };
}
