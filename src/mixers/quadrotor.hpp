/**
 * Copyright (C) 2024 Simon D. Levy
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

static void mixQuadrotor(const demands_t & demands, float motors[], uint8_t & count)
{
    auto t = demands.thrust;
    auto r = demands.roll;
    auto p = demands.pitch;
    auto y = demands.yaw;

    motors[0] = t - r + p + y;
    motors[1] = t - r - p - y;
    motors[2] = t + r - p + y;
    motors[3] = t + r + p - y;

    count = 4;
}


