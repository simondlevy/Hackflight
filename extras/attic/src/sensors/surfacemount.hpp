/*
   Abstract class for surface-mounted sensors (IMU, barometer)

   Copyright (c) 2018 Simon D. Levy
   
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "sensor.hpp"
#include "imu.hpp"

namespace hf {

    class SurfaceMountSensor : public Sensor {

        /*
           The most common aeronautical convention defines roll as acting about
           the longitudinal axis, positive with the starboard (right) wing
           down. Yaw is about the vertical body axis, positive with the nose to
           starboard. Pitch is about an axis perpendicular to the longitudinal
           plane of symmetry, positive nose up.

           https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)

           https://emissarydrones.com/what-is-roll-pitch-and-yaw
        */


        friend class Hackflight;

        protected:

            IMU * imu = NULL;

    };  // class SurfaceMountSensor

} // namespace
