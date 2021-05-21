/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixers/quad.hpp"
#include <rft_motors/realmotors/servo.hpp>
#include <Servo.h>

namespace hf {

    class CoaxialMixer : public QuadMixer {

        public:

            CoaxialMixer(rft::ServoMotor * servo1, rft::ServoMotor * servo2, rft::Motor * rotor1, rft::Motor * rotor2) 
                : QuadMixer(servo1, servo2, rotor1, rotor2)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = {  0, +1,  0,  0 };    // servo 1
                motorDirections[1] = {  0,  0, +1,  0 };    // servo 2
                motorDirections[2] = { +1,  0,  0, +1 };    // rotor 1
                motorDirections[3] = { +1,  0,  0, -1 };    // rotor 2
             }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class CoaxialMixer

} // namespace
