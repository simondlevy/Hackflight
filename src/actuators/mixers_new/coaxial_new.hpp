/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixer_new.hpp"
#include "motor_new.hpp"

#include <Servo.h>

namespace hf {

    class NewCoaxialMixer : public NewMixer {

        public:

            NewCoaxialMixer(/*NewServoMotor * servo1, NewSerovMotor * servo2, NewMotor * motor3, NewMotor * motor4*/) 
                : NewMixer(4)
            {
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class NewCoaxialMixer

} // namespace
