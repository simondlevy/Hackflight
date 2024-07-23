#pragma once

#include <datatypes.h>

class Mixer {

    public:

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

            motors.m1 = t - r - p  + y;  // Front right
            motors.m2 = t - r + p  - y;  // Back right
            motors.m3 = t + r + p  + y;  // Back left
            motors.m4 = t + r - p  - y;  // Front left
        }

};
