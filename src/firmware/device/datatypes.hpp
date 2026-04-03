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

#pragma once

namespace hf {

    class ThreeAxisRaw {

        public:

            int16_t x;
            int16_t y;
            int16_t z;

            ThreeAxisRaw() = default;

            ThreeAxisRaw(const int16_t x, const int16_t y, const int16_t z) 
                : x(x), y(y), z(z) {}

            ThreeAxisRaw& operator=(const ThreeAxisRaw& other) = default;
    };

}

