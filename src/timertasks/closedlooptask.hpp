/*
   Timer task for closed-loop controllers

   Copyright (c) 2021 Simon D. Levy

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
#include "state.hpp"
#include "openloop.hpp"

namespace hf {

    class ClosedLoopTask : public TimerTask {

        friend class Hackflight;

        private:

            // For now, we keep all tasks the same.  At some point it might be
            // useful to investigate, e.g., faster updates for Rate PID than
            // for Level PID.
            static constexpr float FREQ = 300;

            // PID controllers
            ClosedLoopController * _controllers[256] = {NULL};
            uint8_t _controller_count = 0;

            // Other stuff we need
            OpenLoopController * _olc = NULL;
            Actuator * _actuator = NULL;
            State  * _state    = NULL;

        protected:

            ClosedLoopTask(void)
                : TimerTask(FREQ)
            {
                _controller_count = 0;
            }

            void begin(Board * board, OpenLoopController * olc, Actuator * actuator, State * state)
            {
                TimerTask::begin(board);

                _olc = olc;
                _actuator = actuator;
                _state = state;
            }

            void addClosedLoopController(ClosedLoopController * controller, uint8_t modeIndex) 
            {
                controller->modeIndex = modeIndex;

                _controllers[_controller_count++] = controller;
            }

            virtual void doTask(void) override
            {
                // Start with demands from open-loop controller
                float demands[OpenLoopController::MAX_DEMANDS] = {};
                _olc->getDemands(demands);

                // Each controller is associated with at least one auxiliary switch state
                uint8_t modeIndex = _olc->getModeIndex();

                // Some controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_controller_count; ++k) {

                    ClosedLoopController * controller = _controllers[k];

                    // Some controllers need to be reset based on inactivty
                    // (e.g., throttle down resets PID controller integral)
                    controller->resetOnInactivity(_olc->inactive());

                    if (controller->modeIndex <= modeIndex) {

                        controller->modifyDemands(_state, demands); 

                        if (controller->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain controllers
                _board->flashLed(shouldFlash);

                // Use updated demands to run motors
                if (_state->armed && !_state->failsafe && !_olc->inactive()) {
                    _actuator->run(demands);
                }
             }

    };  // ClosedLoopTask

} // namespace hf
