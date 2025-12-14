/**
 * Copyright (C) 2025 Simon D. Levy
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

class Lidar {

    public:

        static void getSetpoint(
                const uint16_t width,
                const uint16_t height,
                const int16_t * distance_mm,
                demands_t & setpoint)
        {
            // XXX for now we ignore the lidar distances and simply hover in place
            (void)distance_mm;

            setpoint.thrust = 0.5;
        }
};
