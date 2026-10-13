/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>

namespace hf {

    class Rotation {

        public:

            float 
                xx, xy, xz,
                yx, yy, yz,
                zx, zy, zz;

            Rotation() :
                xx(1), xy(0), xz(0), 
                yx(0), yy(1), yz(0), 
                zx(0), zy(0), zz(1) {}

            Rotation(
                    const float xx, const float xy, const float xz,
                    const float yx, const float yy, const float yz,
                    const float zx, const float zy, const float zz)
                :
                    xx(xx), xy(xy), xz(xz), 
                    yx(yx), yy(yy), yz(yz), 
                    zx(zx), zy(zy), zz(zz) {}

            Rotation& operator=(const Rotation& other) = default;
    };

}
