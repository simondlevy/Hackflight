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
 
   Copyright (c) 2019 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixers/quad.hpp"
#include <RFT_motor.hpp>


namespace hf {

    class MixerQuadPlusAP : public QuadMixer {

        public:

            MixerQuadPlusAP(rft::Motor * motor1, rft::Motor * motor2, rft::Motor * motor3, rft::Motor * motor4) 
                : QuadMixer(motor1, motor2, motor3, motor4)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1,  0, -1, -1 };    // 1 front
                motorDirections[1] = { +1, -1,  0, +1 };    // 2 right
                motorDirections[2] = { +1,  0, +1, -1 };    // 3 rear
                motorDirections[3] = { +1, +1,  0, +1 };    // 4 left
            }
    };

} // namespace
