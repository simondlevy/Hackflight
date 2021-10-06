/*
   Timer task for closed-loop controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_timertask.hpp"
#include "RFT_state.hpp"

#include "HF_receiver.hpp"
#include "HF_mixer.hpp"

namespace rft {

    class ClosedLoopTask : public TimerTask {

        friend class RFTPure;

        private:

            // PID controllers
            ClosedLoopController * _controllers[256] = {};
            uint8_t _controller_count = 0;

        protected:

            // For now, we keep all tasks the same.  At some point it might be
            // useful to investigate, e.g., faster updates for Rate PID than
            // for Level PID.
            ClosedLoopTask(float freq=300)
                : TimerTask(freq)
            {
                _controller_count = 0;
            }

            void addController(ClosedLoopController * controller,
                               uint8_t modeIndex) 
            {
                controller->modeIndex = modeIndex;

                _controllers[_controller_count++] = controller;
            }

            void update(Board * board,
                        hf::Receiver * receiver,
                        hf::Mixer * mixer,
                        State * state)
            {
                if (!TimerTask::ready(board)) {
                    return;
                }

                // Start with demands from open-loop controller
                float demands[hf::Receiver::MAX_DEMANDS] = {};
                receiver->getDemands(demands);

                // Each controller is associated with at least one auxiliary
                // switch state
                uint8_t modeIndex = receiver->getModeIndex();


                for (uint8_t k=0; k<_controller_count; ++k) {

                    ClosedLoopController * controller = _controllers[k];

                    // Some controllers need to be reset based on inactivty
                    // (e.g., throttle down resets PID controller integral)
                    controller->resetOnInactivity(receiver->inactive());
                }

                // Use updated demands to run motors, allowing
                // mixer to choose whether it cares about
                // open-loop controller being inactive (e.g.,
                // throttle down)
                if (!state->failsafe) {
                    mixer->run(demands, state->armed && !receiver->inactive());
                }

             } // doTask

    };  // ClosedLoopTask

} // namespace rft
