/*
 *   Laser rangefinder (Time-Of-Flight) simulator
 *
 *   Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, in version 3.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

namespace hf {

    class Ranger {

        public:

            static float read(Dynamics & d)
            {
                // See https://www.bitcraze.io/documentation/repository/crazyflie-firmware/
                //   master/images/flowdeck_velocity.png
                return d._x5 / (cos(d._x7) * cos(d._x9));
            }
    };
}
