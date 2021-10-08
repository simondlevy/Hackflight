/*
   Timer task for closed-loop controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_timertask.hpp"
#include "HF_state.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_pidcontroller.hpp"

namespace hf {

    class ControlTask : public TimerTask {

        private:

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

        public:

            // For now, we keep all tasks the same.  At some point it might be
            // useful to investigate, e.g., faster updates for Rate PID than
            // for Level PID.
            ControlTask(float freq=300)
                : TimerTask(freq)
            {
                _controller_count = 0;
            }

            void addController(PidController * controller)
            {
                _controllers[_controller_count++] = controller;
            }

            void update(uint32_t time_usec, Receiver * receiver, Mixer * mixer, State * state)
            {
                // Start with demands from open-loop controller
                float demands[Receiver::MAX_DEMANDS] = {};
                receiver->getDemands(demands);

                bool ready = TimerTask::ready(time_usec);

                // Apply PID controllers to demands
                for (uint8_t k=0; k<_controller_count; ++k) {
                    _controllers[k]->modifyDemands(state->x, demands, ready); 

                }

                // Use updated demands to run motors, allowing
                // mixer to choose whether it cares about
                // open-loop controller being inactive (e.g.,
                // throttle down)
                if (/*ready &&*/ !state->failsafe) {
                    mixer->run(demands, state->armed && !receiver->inactive());
                }

            } // update

    };  // class ControlTask

} // namespace hf
