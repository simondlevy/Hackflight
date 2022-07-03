/*
   Mixer subclass for quadrotors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_mixer.hpp"

#include "../hf_motors/rotary.hpp"

namespace hf {

    class MixerQuad : public Mixer {

        protected:

            MixerQuad(
                    RotaryMotor * motor1,
                    RotaryMotor * motor2,
                    RotaryMotor * motor3,
                    RotaryMotor * motor4)
            {
                Mixer::addMotor(motor1);
                Mixer::addMotor(motor2);
                Mixer::addMotor(motor3);
                Mixer::addMotor(motor4);
            }
    };

} // namespace hf
