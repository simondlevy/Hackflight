/*
   Timer task for mixer

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "timertask.hpp"
#include "mixer.hpp"

namespace hf {

    class MixerTask : public TimerTask {

        private:

            static constexpr float FREQ = 330;

            demands_t * _demands = NULL;

            bool * _failsafe = NULL;

        protected:

            virtual void doTask(void) override
            {
                if (_state->armed && !*_failsafe && !_receiver->throttleIsDown()) {
                    _mixer->runArmed(_demands);
                }
            }

        public:

            MixerTask(void)
                : TimerTask(FREQ)
            {
            }

            void init(Board * board, state_t * state, Mixer * mixer, Receiver * receiver, demands_t * demands, bool * failsafe) 
            {
                TimerTask::init(board, state, mixer, receiver);

                _demands = demands;
                _failsafe = failsafe;
            }

    };  // MixerTask

} // namespace hf
