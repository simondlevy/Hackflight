/**
 * Setpoint from rangedfinder
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

class RangefinderSetpoint {

    public:

        static void run(const int * rangefinder_distances_mm,
                demands_t & setpoint)
        {
            const int * d = rangefinder_distances_mm;

            // Look for clear (infinity reading) in center of 1x8 readings
            const bool center_is_clear = d[3] == -1 && d[4] == -1;

            // If clear, pitch forward
            setpoint.pitch = center_is_clear ? 0.4 : 0;

            // Otherwise, yaw rightward
            setpoint.yaw = center_is_clear ? 0 : 0.2;
        }        
};
