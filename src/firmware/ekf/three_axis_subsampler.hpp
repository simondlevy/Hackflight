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

            float conversionFactor;
            IMU::ThreeAxis sum;
            IMU::ThreeAxis subSample;
            uint32_t count;

            ThreeAxisSubSampler() = default;

            ThreeAxisSubSampler& operator=(const ThreeAxisSubSampler& other) = default;
 
            ThreeAxisSubSampler
                (const float conversionFactor,
                 const IMU::ThreeAxis sum={},
                 const IMU::ThreeAxis subSample={},
                 const uint32_t count={})
                :
                    conversionFactor(conversionFactor),
                    sum(sum),
                    subSample(subSample),
                    count(count) {}

            static auto accumulate(const ThreeAxisSubSampler & subSampler,
                    const IMU::ThreeAxis & sample) -> ThreeAxisSubSampler
            {
                const auto sum = subSampler.sum;

                return ThreeAxisSubSampler(
                        subSampler.conversionFactor,
                        IMU::ThreeAxis(
                            sum.x + sample.x,
                            sum.y + sample.y,
                            sum.z + sample.z),
                        subSampler.subSample,
                        subSampler.count + 1);
            }

            static auto finalize (const ThreeAxisSubSampler &
                    subSampler)-> ThreeAxisSubSampler
            {
                const auto count = subSampler.count;
                const auto sum = subSampler.sum;
                const auto conversionFactor = subSampler.conversionFactor;

                return subSampler.count > 0 ?

                    ThreeAxisSubSampler(
                            conversionFactor,
                            IMU::ThreeAxis(), // sum
                            IMU::ThreeAxis(
                                sum.x * conversionFactor / count,
                                sum.y * conversionFactor / count,
                                sum.z * conversionFactor / count),
                            0) :              // count

                        subSampler;
            }

    };
}
