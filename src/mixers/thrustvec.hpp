/*
   Mixer subclass for thrust vectoring

   Copyright (c) 2021 Simon D. Levy

   MIT Press
 */

#pragma once

#include "mixer.hpp"

#include "mixer.hpp"

namespace hf {

    class MixerThrustVector : public Mixer {

        public:

            MixerThrustVector(rft::Motor * motors) 
                : Mixer(motors, 4)
            {
                //                     Th   Rl   Pi  Yw
                motorDirections[0] = { +1,  0,   0, -1 };   // rotor 1
                motorDirections[1] = { +1,  0,   0, +1 };   // rotor 2
                motorDirections[2] = {  0, +1,   0,  0 };   // servo 1
                motorDirections[3] = {  0,  0 , -1,  0 };   // servo 2
             }

        protected:

            virtual float constrainMotorValue(uint8_t index, float value) override
            {
                return index < 2 ? Mixer::constrainMotorValue(index, value) : value;
            }

    };

} // namespace
