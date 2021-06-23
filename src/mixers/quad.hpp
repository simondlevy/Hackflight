/*
   Mixer subclass for quadrotors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "mixer.hpp"

#include <rft_motors/rotary.hpp>

namespace hf {

    class MixerQuad : public Mixer {

        protected:

            MixerQuad(
                    rft::RotaryMotor * motor1,
                    rft::RotaryMotor * motor2,
                    rft::RotaryMotor * motor3,
                    rft::RotaryMotor * motor4)
            {
                Mixer::addMotor(motor1);
                Mixer::addMotor(motor2);
                Mixer::addMotor(motor3);
                Mixer::addMotor(motor4);
            }

            // For simulation / testing
            MixerQuad()
            {
            }
    };

} // namespace hf
