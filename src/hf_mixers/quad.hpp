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

            MixerQuad(void)
                : Mixer(4)
            {
            }
    };

} // namespace hf
