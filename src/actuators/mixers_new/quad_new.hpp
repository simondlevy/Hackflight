/*
   Mixer subclass for quadcopters 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixer_new.hpp"
#include "motor_new.hpp"

namespace hf {

    class NewQuadMixer : public NewMixer {

        public:

            NewQuadMixer(NewMotor * motor1, NewMotor * motor2, NewMotor * motor3, NewMotor * motor4) 
                : NewMixer(4)
            {
                NewMotor * motors[4] = {motor1, motor2, motor3, motor4};
                NewMixer::useMotors(motors);
            }

    }; // class NewQuadMixer

} // namespace hf
