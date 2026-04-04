/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2026 Simon D. Levy
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

#include <math.h>

namespace hf {

    class ZRangerFilter {

        private:

            static const uint16_t OUTLIER_LIMIT_MM = 4000;

            // Measurement noise model
            static constexpr float EXP_POINT_A = 2.5;
            static constexpr float EXP_STD_A = 0.0025; 
            static constexpr float EXP_POINT_B = 4.0;
            static constexpr float EXP_STD_B = 0.2;   

        public:

            float distance_m;
            float stdev;

            ZRangerFilter() = default;

            ZRangerFilter& operator=(const ZRangerFilter& other) = default;

            ZRangerFilter(const float distance_m, const float stdev)
                : distance_m(distance_m), stdev(stdev) { }

            static auto step(const ZRangerFilter& filter,
                    const uint16_t distance_mm) -> ZRangerFilter
            {
                static constexpr float EXP_COEFF =
                    logf(EXP_STD_B / EXP_STD_A) / (EXP_POINT_B - EXP_POINT_A);

                const auto distance_m = distance_mm / 1000.f;

                const auto stdev = EXP_STD_A *
                    ( 1 + expf(EXP_COEFF * (distance_m - EXP_POINT_A)));

                return distance_mm < OUTLIER_LIMIT_MM ?
                    ZRangerFilter(distance_m, stdev) :
                    filter;
            }
    };

}
