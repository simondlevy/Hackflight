/*
   Mixer subclass for quadcopters 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixer.hpp"
#include "motor.hpp"

namespace hf {

    class QuadMixer : public Mixer {

        public:

            QuadMixer(Motor * motor1, Motor * motor2, Motor * motor3, Motor * motor4) 
                : Mixer(4)
            {
                Motor * motors[4] = {motor1, motor2, motor3, motor4};
                Mixer::useMotors(motors);
            }

    }; // class QuadMixer

} // namespace hf
