/*
   Mixer subclass for thrust vectoring

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../rft_motors/rotary.hpp"
#include "../rft_motors/servo.hpp"

#include "HF_mixer.hpp"

namespace hf {

    class MixerThrustVector : public Mixer {

        public:

            MixerThrustVector(
                    rft::RotaryMotor * rotor1,
                    rft::RotaryMotor * rotor2,
                    rft::ServoMotor * servo1,
                    rft::ServoMotor * servo2)
            {
                //                     Th   RR   PF  YR
                motorDirections[0] = { +1,  0,   0, +1 };   // rotor 1
                motorDirections[1] = { +1,  0,   0, -1 };   // rotor 2
                motorDirections[2] = {  0, +1,   0,  0 };   // servo 1
                motorDirections[3] = {  0,  0 , +1,  0 };   // servo 2

                Mixer::addMotor(rotor1);
                Mixer::addMotor(rotor2);
                Mixer::addMotor(servo1);
                Mixer::addMotor(servo2);
             }

        protected:

            virtual float constrainMotorValue(uint8_t index, float value) override
            {
                return index < 2 ? Mixer::constrainMotorValue(index, value) : value;
            }

    };

} // namespace
