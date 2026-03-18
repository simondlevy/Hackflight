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
            Vec3 subSample;
            Vec3 sum;
            uint32_t count;

            Vec3SubSampler() = default;

            Vec3SubSampler(
                    const float conversionFactor,
                    const Vec3 & subSample = {},
                    const Vec3 & sum = {},
                    const uint32_t count = 0)
                : 
                    conversionFactor(conversionFactor),
                    subSample(subSample),
                    sum(sum),
                    count(count) {}

            static auto reset(const Vec3SubSampler & ss) -> Vec3SubSampler
            {
                return Vec3SubSampler(ss.conversionFactor);
            }

            static auto accumulate(const Vec3SubSampler & ss,
                    const Vec3 & sample) -> Vec3SubSampler
            {
                return Vec3SubSampler(
                        ss.conversionFactor,
                        ss.subSample,
                        ss.sum + sample,
                        ss.count + 1);
            }

            static auto newreset(const Vec3SubSampler & ss) -> Vec3SubSampler
            {
                return Vec3SubSampler(ss.conversionFactor, ss.subSample);
            }

            static auto newfinalize(const Vec3SubSampler & ss)
                -> Vec3SubSampler
            {
                return ss.count > 0 ?
                    Vec3SubSampler(ss.conversionFactor,
                            ss.sum * ss.conversionFactor / ss.count) :
                    ss;
            }

            ///////////////////////////////////////////////////////////

            static void reset(Vec3SubSampler & ss)
            {
                ss.sum = {};
                ss.count = 0;
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
