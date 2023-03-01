/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>
#include <logic/core/mixers/fixedpitch/quadxbf.h>
#include <logic/core/pids/angle.h>
#include <logic/imu/softquat.h>
#include <logic/logic.h>

#include <vector>

class QuadLogic : public Logic {

    private:

        AnglePidController anglePid = AnglePidController(
                1.441305,     // Rate Kp
                48.8762,      // Rate Ki
                0.021160,     // Rate Kd
                0.0165048,    // Rate Kf
                0.0); // 3.0; // Level Kp

        Mixer mixer = QuadXbfMixer::make();

        SoftQuatImu imu = SoftQuatImu(Imu::rotate270);

        std::vector<PidController *> pids = {&anglePid};

    public:

        QuadLogic(void) 
            : Logic(&imu, pids, mixer, 168)
        {
        }
}; 
