/**
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

#include "flydar.hpp"

static constexpr float TRAVEL_AFTER_CLEAR_SEC = 1;

bool Flydar::succeeded(
        const int frame, 
        const int * rangefinder_distances_mm, 
        const int rangefinder_size)
{
    for (int i=0; i<rangefinder_size; ++i) {
        if (rangefinder_distances_mm[i] != -1) {
            return false;
        }
    }

    static int _cleared_at_frame;

    if (_cleared_at_frame == 0) {
        _cleared_at_frame = frame;
    }

    else if ((frame - _cleared_at_frame)/FRAME_RATE_HZ > TRAVEL_AFTER_CLEAR_SEC) {
        return true;
    }

    return false;
}


