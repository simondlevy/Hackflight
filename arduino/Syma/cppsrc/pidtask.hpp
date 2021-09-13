/*
   Timer task for closed-loop controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "timertask.hpp"
#include "state.hpp"
#include "openloop.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class PidControlTask : public TimerTask {

        friend class Hackflight;

        private:

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

        protected:

            // For now, we keep all tasks the same.  At some point it might be
            // useful to investigate, e.g., faster updates for Rate PID than
            // for Level PID.
            PidControlTask(float freq=300)
                : TimerTask(freq)
            {
                _controller_count = 0;
            }

            void addController(PidController * controller,
                               uint8_t modeIndex) 
            {
                controller->modeIndex = modeIndex;

                _controllers[_controller_count++] = controller;
            }

            void update(Board * board,
                        OpenLoopController * olc,
                        Mixer * mixer,
                        State * state)
            {
                if (!TimerTask::ready(board)) {
                    return;
                }

                // Start with demands from open-loop controller
                float demands[OpenLoopController::MAX_DEMANDS] = {};
                olc->getDemands(demands);

                // Each controller is associated with at least one auxiliary
                // switch state
                uint8_t modeIndex = olc->getModeIndex();

                // Some controllers should cause LED to flash when they're
                // active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_controller_count; ++k) {

                    PidController * controller = _controllers[k];

                    // Some controllers need to be reset based on inactivty
                    // (e.g., throttle down resets PID controller integral)
                    controller->resetOnInactivity(olc->inactive());

                    if (controller->modeIndex <= modeIndex) {

                        controller->modifyDemands(state->x, demands); 

                        if (controller->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain controllers
                board->flashLed(shouldFlash);

                // Use updated demands to run motors, allowing
                // mixer to choose whether it cares about
                // open-loop controller being inactive (e.g.,
                // throttle down)
                if (!state->failsafe) {
                    mixer->run(demands, state->armed && !olc->inactive());
                }

             } // doTask

    };  // PidControlTask

} // namespace hf
