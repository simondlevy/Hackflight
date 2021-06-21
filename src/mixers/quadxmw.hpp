/*
   Mixer subclass for X-configuration quadcopters following the MultiWii numbering convention:

    4cw   2ccw
       \ /
        ^
       / \
    3ccw  1cw
 
   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "board.hpp"
#include "datatypes.hpp"
#include "mixer.hpp"

namespace hf {

    class MixerQuadXMW : public Mixer {

        public:

            MixerQuadXMW(Motor * motors) 
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
