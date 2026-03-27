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

#include <firmware/datatypes.hpp>

namespace hf {

    class ImuSubSampler {

        public:

            float conversionFactor;
            Vec3 subSample;
            Vec3 sum;
            uint32_t count;

            ImuSubSampler() = default;

            ImuSubSampler(
                    const float conversionFactor,
                    const Vec3 & subSample = {},
                    const Vec3 & sum = {},
                    const uint32_t count = 0)
                : 
                    conversionFactor(conversionFactor),
                    subSample(subSample),
                    sum(sum),
                    count(count) {}

            static auto accumulate(const ImuSubSampler & ss,
                    const Vec3 & sample) -> ImuSubSampler
            {
                return ImuSubSampler(
                        ss.conversionFactor,
                        ss.subSample,
                        ss.sum + sample,
                        ss.count + 1);
            }

            static auto finalize(const ImuSubSampler & ss)
                -> ImuSubSampler
            {
                return ss.count > 0 ?
                    ImuSubSampler(ss.conversionFactor,
                            ss.sum * ss.conversionFactor / ss.count) :
                    ss;
            }
    };
}
