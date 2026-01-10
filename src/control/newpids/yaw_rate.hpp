/**
 *
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

#include <num.hpp>

class YawRateController {

    public:

          static float run(
                 const float dt,
                 const float dpsi_actual,
                 const float dpsi_target)
        {
            static float _integral;

            const auto error = dpsi_target - dpsi_actual;

            _integral = Num::fconstrain(_integral + error * dt, ILIMIT);

            return Num::fconstrain(KP * error + KI * _integral, OUTPUT_LIMIT);
        }

    private:

         static constexpr float KP = 120;
         static constexpr float KI = 16.7;
         static constexpr float ILIMIT = 166.7;
         static constexpr float OUTPUT_LIMIT = INT16_MAX;

         float _integral;
};
