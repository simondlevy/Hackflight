/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixers_new/quad_new.hpp"
#include "motors_new/servo.hpp"

#include <Servo.h>

namespace hf {

    class NewCoaxialMixer : public NewQuadMixer {

        public:

            NewCoaxialMixer(NewServoMotor * servo1, NewServoMotor * servo2, NewMotor * motor1, NewMotor * motor2) 
                : NewQuadMixer(servo1, servo2, motor1, motor2)
            {
                //                     Th  RR  PF  YR
                // motorDirections[0] = { +1, -1, +1, -1 };    // 1 right rear
                // motorDirections[1] = { +1, -1, -1, +1 };    // 2 right front
                // motorDirections[2] = { +1, +1, +1, +1 };    // 3 left rear
                // motorDirections[3] = { +1, +1, -1, -1 };    // 4 left front
             }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class NewCoaxialMixer

} // namespace
