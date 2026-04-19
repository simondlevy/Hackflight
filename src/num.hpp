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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <datatypes.hpp>

class Num {

    public:

        // Small number epsilon, to prevent dividing by zero
        static constexpr float EPSILON = 1e-6f;

        static constexpr float RAD2DEG = 180.0f / M_PI;
        static constexpr float DEG2RAD = M_PI / 180.0f;

        static auto fconstrain(const float val, const float maxabs) -> float
        {
            return val < -maxabs ? -maxabs : val > maxabs ? maxabs : val;
        }

        static auto fconstrain(float value, const float minVal,
                const float maxVal) -> float
        {
            return fminf(maxVal, fmaxf(minVal,value));
        }

        static auto cap_angle(const float angle) -> float
        {
            float result = angle;

            while (result > 180.0f) {
                result -= 360.0f;
            }

            while (result < -180.0f) {
                result += 360.0f;
            }

            return result;
        }

        static auto rescale(
                const float value,
                const float oldmin, 
                const float oldmax, 
                const float newmin, 
                const float newmax) -> float
        {
            return (value - oldmin) / (oldmax - oldmin) * 
                (newmax - newmin) + newmin;
        }

};
