/*
  Motor mixers for Hackflight
 
  Copyright (C) 2024 Simon D. Levy
 
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

#include <hackflight.hpp>

class Mixer {

    public:

        /*
         Betaflight QuadX:

                    4     2
                       x
                    3     1

         */
        static void runBF(const demands_t & demands, quad_motors_t & motors)
        {
            const auto t = demands.thrust;
            const auto r = demands.roll;
            const auto p = demands.pitch;
            const auto y = demands.yaw;

            motors.m1 = t - r + p  - y;
            motors.m2 = t - r - p  + y;
            motors.m3 = t + r + p  + y;
            motors.m4 = t + r - p  - y;
        }        
        
        /*
         Crazyflie QuadX:

                    4     1
                       x
                    3     2

         */
        static void runCF(const demands_t & demands, quad_motors_t & motors)
        {
            const auto t = demands.thrust;
            const auto r = demands.roll;
            const auto p = demands.pitch;
            const auto y = demands.yaw;

            motors.m1 = t - r - p  + y;
            motors.m2 = t - r + p  - y;
            motors.m3 = t + r + p  + y;
            motors.m4 = t + r - p  - y;
        }

};
