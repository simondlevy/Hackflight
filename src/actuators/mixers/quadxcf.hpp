/*
   Mixer subclass for X-configuration quadcopters following the Cleanflight numbering convention:

    4cw   2ccw
       \ /
        ^
       / \
    3ccw  1cw
 
   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "datatypes.hpp"
#include "actuators/mixer.hpp"

namespace hf {

    class MixerQuadXCF : public Mixer {

        public:

            MixerQuadXCF(void) 
                : Mixer(4)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, +1, +1 };    // 1 right rear
                motorDirections[1] = { +1, -1, -1, -1 };    // 2 right front
                motorDirections[2] = { +1, +1, +1, -1 };    // 3 left rear
                motorDirections[3] = { +1, +1, -1, +1 };    // 4 left front
            }
    };

} // namespace
