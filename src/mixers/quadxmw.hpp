/*
   Mixer subclass for X-configuration quadcopters following the MultiWii numbering convention:

   4cw   2ccw
      \ /
       ^
      / \
   3ccw  1cw

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "mixer.hpp"

#include <rft_motors/rotary.hpp>

namespace hf {

    class MixerQuadXMW : public Mixer {

        protected:

            void construct(void)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, +1, +1 };    // 1 right rear
                motorDirections[1] = { +1, -1, -1, -1 };    // 2 right front
                motorDirections[2] = { +1, +1, +1, -1 };    // 3 left rear
                motorDirections[3] = { +1, +1, -1, +1 };    // 4 left front
            }

        public:

            MixerQuadXMW(
                    rft::RotaryMotor * motor1,
                    rft::RotaryMotor * motor2,
                    rft::RotaryMotor * motor3,
                    rft::RotaryMotor * motor4)
            {
                Mixer::addMotor(motor1);
                Mixer::addMotor(motor2);
                Mixer::addMotor(motor3);
                Mixer::addMotor(motor4);

                construct();
            }

            // For simulation / testing
            MixerQuadXMW(void)
            {
                construct();
            }
    };

} // namespace hf
