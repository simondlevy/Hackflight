/*
   Mixer subclass for X-configuration quadcopters following the ArduPilot numbering convention:

    3cw   1ccw
       \ /
        ^
       / \
    2ccw  4cw
 
   Copyright (C) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_motor.hpp>

#include "mixer.hpp"

namespace hf {

    class MixerQuadXAP : public Mixer {

        public:

            MixerQuadXAP(rft::Motor * motors) 
                : Mixer(motors, 4)
            {
                //                     Th  Rl  Pi  Yw
                motorDirections[0] = { +1, -1, +1, +1 };    // 1 right front
                motorDirections[1] = { +1, +1, -1, +1 };    // 2 left rear
                motorDirections[2] = { +1, +1, +1, -1 };    // 3 left front
                motorDirections[3] = { +1, -1, -1, -1 };    // 4 right rear
            }
    };

} // namespace
