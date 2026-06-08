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

#include <firmware/imu/three_axis.hpp>

namespace hf {

    class ThreeAxisSubSampler {

        public:

            float conversion_factor;
            ThreeAxis sum;
            ThreeAxis sub_sample;
            uint32_t count;

            ThreeAxisSubSampler() = default;

            ThreeAxisSubSampler& operator=(const ThreeAxisSubSampler& other) = default;
 
            ThreeAxisSubSampler
                (const float conversion_factor,
                 const ThreeAxis sum={},
                 const ThreeAxis sub_sample={},
                 const uint32_t count={})
                :
                    conversion_factor(conversion_factor),
                    sum(sum),
                    sub_sample(sub_sample),
                    count(count) {}

            static auto accumulate(const ThreeAxisSubSampler & sub_sampler,
                    const ThreeAxis & sample) -> ThreeAxisSubSampler
            {
                const auto sum = sub_sampler.sum;

                return ThreeAxisSubSampler(
                        sub_sampler.conversion_factor,
                        ThreeAxis(
                            sum.x + sample.x,
                            sum.y + sample.y,
                            sum.z + sample.z),
                        sub_sampler.sub_sample,
                        sub_sampler.count + 1);
            }

            static auto finalize (const ThreeAxisSubSampler & sub_sampler
                    )-> ThreeAxisSubSampler
            {
                const auto count = sub_sampler.count;
                const auto sum = sub_sampler.sum;
                const auto conversion_factor = sub_sampler.conversion_factor;

                return sub_sampler.count > 0 ?

                    ThreeAxisSubSampler(
                            conversion_factor,
                            ThreeAxis(), // sum
                            ThreeAxis(
                                sum.x * conversion_factor / count,
                                sum.y * conversion_factor / count,
                                sum.z * conversion_factor / count),
                            0) :        // count

                        sub_sampler;
            }

    };
}
