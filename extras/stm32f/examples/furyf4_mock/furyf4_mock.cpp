/*
   Sketch for FURYF4 board with mock receiver

   Copyright (c) 2019 Simon D. Levy

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
#include <receivers/mock.hpp>
#include "furyf4.h"

static hf::Hackflight h;

extern "C" {

#include "time.h"

    void setup(void)
    {
        hf::Rate * ratePid = new hf::Rate(
                0.05f, // Gyro cyclic P
                0.00f, // Gyro cyclic I
                0.00f, // Gyro cyclic D
                0.10f, // Gyro yaw P
                0.01f, // Gyro yaw I
                8.58); // Demands to rate

        h.addPidController(level, 1);

        h.init(new FuryF4(), new hf::MockReceiver(), new hf::MixerQuadX(), ratePid);
    }

    void loop(void)
    {
        h.update();
    }

} // extern "C"
