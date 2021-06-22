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

            MixerQuadXMW(motor_type_t mtype, uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
                : Mixer(mtype)
            {
                construct();
            }

            MixerQuadXMW(void)
            {
                construct();
            }
    };

    class BrushedMixerQuadXMW : public MixerQuadXMW {

        public:

            BrushedMixerQuadXMW(uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
                : MixerQuadXMW(Mixer::BRUSHED, m1_pin, m2_pin, m3_pin, m4_pin)
            {
            }
    };

    class BrushlessMixerQuadXMW : public MixerQuadXMW {

        public:

            BrushlessMixerQuadXMW(uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
                : MixerQuadXMW(Mixer::BRUSHLESS, m1_pin, m2_pin, m3_pin, m4_pin)
            {
            }
    };

} // namespace hf
