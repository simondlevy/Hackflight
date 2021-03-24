/*
   quadplusap.hpp : Mixer subclass for + configuration quadcopters following the
   ArduPilot numbering convention:

          1cw
           |
           |
    4ccw - ^ - 2ccw
           |
           |
          3cw
 
   Copyright (C) 2021 Simon D. Levy

   MIT Press
 */

#pragma once

#include <RFT_motor.hpp>

#include "mixer.hpp"

namespace hf {

    class MixerQuadPlusAP : public Mixer {

        public:

            MixerQuadPlusAP(rft::Motor * motors) 
                : Mixer(motors, 4)
            {
                //                     Th  Rl  Pi  Yw
                // XXX Pitch and yaw are probably inverted!
                motorDirections[0] = { +1,  0, -1, +1 };    // 1 front
                motorDirections[1] = { +1, -1,  0, -1 };    // 2 right
                motorDirections[2] = { +1,  0, +1, +1 };    // 3 rear
                motorDirections[3] = { +1, +1,  0, -1 };    // 4 left
            }
    };

} // namespace
