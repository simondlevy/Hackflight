/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_mixer.hpp"
#include "HF_state.hpp"
#include "HF_motors.hpp"
#include "HF_debugger.hpp"

namespace hf {

    class HackflightPure {

        protected:

            // Essentials
            Mixer * _mixer = NULL;
            state_t _state = {};

        public:

            HackflightPure(Mixer * mixer)
            {
                _mixer = mixer;
            }

            void update(float tdmd, float rdmd, float pdmd, float ydmd, motors_t & motors)
            {
                demands_t demands = {tdmd, rdmd, pdmd, ydmd};

                // Use updated demands to run motors
                _mixer->run(demands, motors);
            }

    }; // class HackflightPure

} // namespace hf
