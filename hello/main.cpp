/*
   main routine for "HelloHackflight" demo

   Copyright (C) Simon D. Levy 2017

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

#include <hackflight.hpp>

#include <stdio.h>
#include <stdint.h>

int main(int argc, char ** argv)
{
    int16_t accel[3];
    int16_t gyro[3];
    float rcChannels[4];

    Hackflight hackflight;

    hackflight.initialize();

    hackflight.arm();

    for (int k=0; k<3; ++k) {
        accel[k] = 0;
        gyro[k] = 0;
    }

    rcChannels[0] = 1500;   // roll
    rcChannels[1] = 1500;   // pitch
    rcChannels[2] = 1500;   // yaw
    rcChannels[3] = 1000;   // throttle
    
    hackflight.setAccelReading(accel);
    hackflight.setGyroReading(gyro);
    hackflight.setRC(rcChannels, 4);

    return 0;
}
