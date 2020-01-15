/*
   Timer task for PID controllers

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

namespace hf {

    class PidsTask : public TimerTask {

        private:

            static constexpr float FREQ = 300;

        protected:

            // TimerTask overrides -------------------------------------------------------

            virtual void doTask(void) override
            {
            }


        public:

            PidsTask(void)
                : TimerTask(FREQ)
            {
            }

            void init(Board * board, state_t * state, Mixer * mixer, Receiver * receiver) 
            {
                TimerTask::init(board, state, mixer, receiver);
            }


    };  // PidsTask

} // namespace hf
