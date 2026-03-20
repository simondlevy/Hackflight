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

            auto operator*(const Vec7 & v) -> float
            {
                return _0*v._0+_1*v._1+_2*v._2+_3*v._3+_4*v._4+_5*v._5+_6*v._6;
            }
    };

    class Mat7x7 {

        public:

            Vec7 _0, _1, _2, _3, _4, _5, _6;

            Vec7 _0t, _1t, _2t, _3t, _4t, _5t, _6t;

            Mat7x7() = default;

            Mat7x7(
                    const Vec7 & _0,
                    const Vec7 & _1,
                    const Vec7 & _2,
                    const Vec7 & _3,
                    const Vec7 & _4,
                    const Vec7 & _5,
                    const Vec7 & _6)
                :
                    _0(_0), _1(_1), _2(_2), _3(_3), _4(_4), _5(_5), _6(_6),

                    _0t(_0._0, _1._0, _2._0, _3._0, _4._0, _5._0, _6._0),
                    _1t(_0._1, _1._1, _2._1, _3._1, _4._1, _5._1, _6._1),
                    _2t(_0._2, _1._2, _2._2, _3._2, _4._2, _5._2, _6._2),
                    _3t(_0._2, _1._3, _2._3, _3._3, _4._3, _5._3, _6._3),
                    _4t(_0._4, _1._4, _2._4, _3._4, _4._4, _5._4, _6._4),
                    _5t(_0._5, _1._5, _2._5, _3._5, _4._5, _5._5, _6._5),
                    _6t(_0._6, _1._6, _2._6, _3._6, _4._6, _5._6, _6._6) {}

            auto operator*(const Mat7x7 & a) -> Mat7x7 
            {
                return Mat7x7 (
                        {_0*a._0t, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0}
                        );
            }
     };
}
