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
                    const float _0,
                    const float _1,
                    const float _2,
                    const float _3,
                    const float _4,
                    const float _5,
                    const float _6)
                : _0(_0), _1(_1), _2(_2), _3(_3), _4(_4), _5(_5), _6(_6) { }
    };
}
