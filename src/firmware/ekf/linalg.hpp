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

#include <datatypes.hpp>

namespace hf {

    class Vec7 {

        public:

            float _0, _1, _2, _3, _4, _5, _6;

            Vec7() = default;

            Vec7(
                    const float _0,  // Z
                    const float _1,  // VX
                    const float _2,  // VY
                    const float _3,  // VZ
                    const float _4,  // D0
                    const float _5,  // D1
                    const float _6)  // D2
                : _0(_0), _1(_1), _2(_2), _3(_3), _4(_4), _5(_5), _6(_6) { }
    };

    class Mat7x7 {

        public:

            Vec7 _0, _1, _2, _3, _4, _5, _6;

            Mat7x7() = default;

            Mat7x7(
                    const Vec7 & _0,
                    const Vec7 & _1,
                    const Vec7 & _2,
                    const Vec7 & _3,
                    const Vec7 & _4,
                    const Vec7 & _5,
                    const Vec7 & _6)
                : _0(_0), _1(_1), _2(_2), _3(_3), _4(_4), _5(_5), _6(_6) { }
    };}
