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

            static const uint16_t kOutlierLimitMm = 4000;

            // Measurement noise model
            static constexpr float kExpPointA = 2.5;
            static constexpr float kExpStdA = 0.0025; 
            static constexpr float kExpPointB = 4.0;
            static constexpr float kExpStdB = 0.2;   

        public:

            float distance_m;
            float stdev;

            ZRangerFilter() = default;

            ZRangerFilter& operator=(const ZRangerFilter& other) = default;

            ZRangerFilter(const float distance_m, const float stdev)
                : distance_m(distance_m), stdev(stdev) { }

            static auto Update(const ZRangerFilter& filter,
                    const uint16_t distance_mm) -> ZRangerFilter
            {
                static constexpr float kExpCOEFF =
                    logf(kExpStdB / kExpStdA) / (kExpPointB - kExpPointA);

                const auto distance_m = distance_mm / 1000.f;

                const auto stdev = kExpStdA *
                    ( 1 + expf(kExpCOEFF * (distance_m - kExpPointA)));

                return distance_mm < kOutlierLimitMm ?
                    ZRangerFilter(distance_m, stdev) :
                    filter;
            }
    };

}
