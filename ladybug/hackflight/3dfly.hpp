/*
   3dfly.hpp : 3DFly PIDs for Ladybug Flight Controller 

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

#include "ladybug.hpp"

namespace hf {

    const Config& Ladybug::getConfig(void)
    {
        // PIDs
        config.stabilize.levelP         = 0.20f;

        config.stabilize.ratePitchrollP = 0.225f;
        config.stabilize.ratePitchrollI = 0.001875f;
        config.stabilize.ratePitchrollD = 0.375f;

        config.stabilize.yawP           = 1.0625f;
        config.stabilize.yawI           = 0.005625f;

        // "Software trim"
        config.stabilize.softwareTrim[AXIS_ROLL]  = 0;
        config.stabilize.softwareTrim[AXIS_PITCH] = 0;
        config.stabilize.softwareTrim[AXIS_YAW]   = 0;

        // Altitude-hold XXX not tuneable; belongs elsewhere
        config.altitude.accel.oneG = 2048;

        return config;
    }

} // namespace
