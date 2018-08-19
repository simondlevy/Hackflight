/*
   Sketch for SP Racing F3 board with Spektrum DSMX receiver

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

#include <hackflight.hpp>
#include <mixers/quadx.hpp>
#include "dsmx.h"

#include "spracingf3.h"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static hf::Hackflight h;

// Called by main.c, so must be treated as C
extern "C" {

    // NB: these must be declared here for sketch to work!
    static SPRacingF3 * board;
    static DSMX_Receiver * rc;

    static float motorval;
    static int8_t motordir;

    void setup(void)
    {
        board = new SPRacingF3();

           hf::Stabilizer * stabilizer = new hf::Stabilizer(
           0.10f,      // Level P
           0.125f,     // Gyro cyclic P
           0.001875f,  // Gyro cyclic I
           0.175f,     // Gyro cyclic D
           0.625f,    // Gyro yaw P
           0.005625f); // Gyro yaw I
         
        rc = new DSMX_Receiver(
                CHANNEL_MAP,
                .005f,  // roll trim
                .01f,  // pitch trim
                0.f);   // yaw trim

        // Initialize Hackflight firmware
        h.init(board, rc, new hf::MixerQuadX(), stabilizer);

        motorval = 0;
        motordir = +1;
    }

    void loop(void)
    {
        h.update();

        /*
        hf::Debug::printfloat(motorval);
        hf::Debug::printf("\n");

        board->writeMotor(0, motorval);

        motorval += .01 * motordir;

        if (motorval >= 1.0) {
            motordir = -1;
        }

        if (motorval <= 0.0) {
            motordir = +1;
        }

        board->delaySeconds(.1);*/
    }

} // extern "C"
