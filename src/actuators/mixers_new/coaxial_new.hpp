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
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class NewCoaxialMixer

} // namespace
