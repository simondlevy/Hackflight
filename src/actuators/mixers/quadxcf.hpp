/*
   Mixer subclass for X-configuration quadcopters following the Cleanflight numbering convention:

    4cw   2ccw
       \ /
        ^
       / \
    3ccw  1cw
 
   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "board.hpp"
#include "actuators/mixer.hpp"

namespace hf {

    class MixerQuadXCF : public Mixer {

        public:

            MixerQuadXCF(rft::Motor * motors) 
                : Mixer(motors, 4)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, +1, +1 };    // 1 right rear
                motorDirections[1] = { +1, -1, -1, -1 };    // 2 right front
                motorDirections[2] = { +1, +1, +1, -1 };    // 3 left rear
                motorDirections[3] = { +1, +1, -1, +1 };    // 4 left front
            }
    };

} // namespace
