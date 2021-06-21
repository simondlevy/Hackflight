/*
   Mixer subclass for X-configuration octocopters following the ArduPilot numbering convention:

        5CCW   1CW
                  
    7CW           3CCW
                   
             ^      
                   
    6CW           8CCW
                   
        2CCW   4CW
 
   Copyright (c) 2019 Simon D. Levy

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

    class MixerOctoXAP : public Mixer {

        public:

            MixerOctoXAP(void) 
                : Mixer(8)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, -1, +1 }; // 1
                motorDirections[1] = { +1, +1, +1, +1 }; // 2    
                motorDirections[2] = { +1, -1, -1, -1 }; // 3 
                motorDirections[3] = { +1, -1, +1, -1 }; // 4 
                motorDirections[4] = { +1, +1, -1, -1 }; // 5 
                motorDirections[5] = { +1, +1, +1, -1 }; // 6 
                motorDirections[6] = { +1, +1, -1, +1 }; // 7  
                motorDirections[7] = { +1, -1, +1, +1 }; // 8 
            }
    };

} // namespac6
