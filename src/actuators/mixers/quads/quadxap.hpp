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

#include "actuators/mixers/quad.hpp"
#include <RFT_motor.hpp>

namespace hf {

    class MixerQuadXAP : public QuadMixer {

        public:

            MixerQuadXAP(rft::Motor * motor1, rft::Motor * motor2, rft::Motor * motor3, rft::Motor * motor4) 
                : QuadMixer(motor1, motor2, motor3, motor4)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, -1, +1 };    // 1 right front
                motorDirections[1] = { +1, +1, +1, +1 };    // 2 left rear
                motorDirections[2] = { +1, +1, -1, -1 };    // 3 left front
                motorDirections[3] = { +1, -1, +1, -1 };    // 4 right rear
            }
    };

} // namespace
