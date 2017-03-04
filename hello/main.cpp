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
#include <time.h>

static void delay(uint32_t msec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = msec * 1e6;

    nanosleep(&ts, NULL);
}

int main(int argc, char ** argv)
{
    int16_t accel[3];
    int16_t gyro[3];
    float rcChannels[4];
    float controls[4];

    Hackflight hackflight;

    hackflight.initialize();

    hackflight.arm();

    while (true) {

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

        hackflight.setTime(clock());

        hackflight.getControls(controls, 4);

        for (int k=0; k<4; ++k) {
            printf("%f ", controls[k]);
        }
        printf("\n");

        delay(1000);
    }

    return 0;
}
