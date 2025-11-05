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

class ByteScaling {

    public:

        static float byte2float(const uint8_t val, const float min, const float max)
        {
            return min + (float)val / 255 * (max - min);
        }

        static float byte2float(const uint8_t val, const float max)
        {
            return -max + (float)val / 255 * 2 * max;
        }

        static uint8_t float2byte(const float val, const float min, const float max)
        {
            return (uint8_t)(255 * (val - min) / (max - min));
        }

        static uint8_t float2byte(const float val, const float max)
        {
            return (uint8_t)(255 * (val + max) / (2*max));
        }
};
