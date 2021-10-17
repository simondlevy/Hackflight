/*
   Mixer subclass for X-configuration quadcopters following the ArduPilot numbering convention:

    3cw   1ccw
       \ /
        ^
       / \
    2ccw  4cw
 
   Copyright (c) 2019 Simon D. Levy

   MIT License
 */

#pragma once

#include "../quad.hpp"

namespace hf {

    class MixerQuadXAP : public MixerQuad {

        public:

            MixerQuadXAP(void)
                : MixerQuad()
            {
                //                     Th  RR  PF  YR
                spins[0] = { +1, -1, -1, +1 };    // 1 right front
                spins[1] = { +1, +1, +1, +1 };    // 2 left rear
                spins[2] = { +1, +1, -1, -1 };    // 3 left front
                spins[3] = { +1, -1, +1, -1 };    // 4 right rear
            }

     }; // class MixerQuadXAP

} // namespace hf
