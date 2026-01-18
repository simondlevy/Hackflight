/**
 * Generic multirotor mixer function
 *
 * Copyright (C) 2026 Simon D. Levy
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

#include <datatypes.h>

class RotorMixer {

    public:

        static float * mix(
                const demands_t & demands,
                const int8_t * roll,
                const int8_t * pitch,
                const int8_t * yaw,
                const int8_t count)
        {
            static float motors[MAX_MOTOR_COUNT];
            mix(demands, roll, pitch, yaw, count, motors);
            return motors;
        }        
        
        static void mix(
                const demands_t & demands,
                const int8_t * roll,
                const int8_t * pitch,
                const int8_t * yaw,
                const int8_t count,
                float motors[])
        {
            for (uint8_t k=0; k<count; ++k) {
                motors[k] =
                    demands.thrust +
                    demands.roll * roll[k] +
                    demands.pitch * pitch[k] +
                    demands.yaw * yaw[k];
            }
        }
};


