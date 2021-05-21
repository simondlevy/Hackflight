/*
   Mixer subclass for quadcopters 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "actuators/mixer.hpp"

namespace hf {

    class QuadMixer : public Mixer {

        public:

            QuadMixer(rft::Motor * motor1, rft::Motor * motor2, rft::Motor * motor3, rft::Motor * motor4) 
                : Mixer(4)
            {
                rft::Motor * motors[4] = {motor1, motor2, motor3, motor4};
                Mixer::useMotors(motors);
            }

    }; // class QuadMixer

} // namespace hf
