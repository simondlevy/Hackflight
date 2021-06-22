/*
   Timer task for PID controllers

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_timertask.hpp>
#include <RFT_actuator.hpp>
#include <RFT_openloop.hpp>

namespace hf {

    class ClosedLoopTask : public rft::TimerTask {

        friend class Hackflight;

        private:

            // For now, we keep all PID timer tasks the same.  At some point it might be useful to 
            // investigate, e.g., faster updates for Rate PID than for Level PID.
            static constexpr float FREQ = 300;

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

            // Other stuff we need
            rft::OpenLoopController * _olc = NULL;
            rft::Actuator * _actuator = NULL;
            rft::State  * _state    = NULL;

        protected:

            ClosedLoopTask(void)
                : rft::TimerTask(FREQ)
            {
                _controller_count = 0;
            }

            void begin(rft::Board * board, rft::OpenLoopController * olc, rft::Actuator * actuator, rft::State * state)
            {
                rft::TimerTask::begin(board);

                _olc = olc;
                _actuator = actuator;
                _state = state;
            }

            void addController(PidController * controller, uint8_t modeIndex) 
            {
                controller->modeIndex = modeIndex;

                _controllers[_controller_count++] = controller;
            }

            virtual void doTask(void) override
            {
                // Start with demands from olc, scaling roll/pitch/yaw by constant
                float demands[4] = {};
                _olc->getDemands(demands);

                // Each PID controllers is associated with at least one mode
                uint8_t modeIndex = _olc->getModeIndex();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_controller_count; ++k) {

                    PidController * controller = _controllers[k];

                    // Some PID controllers need to reset their integral when the throttle is down
                    controller->resetOnInactivity(_olc->inactive());

                    if (controller->modeIndex <= modeIndex) {

                        controller->modifyDemands(_state, demands); 

                        if (controller->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);

                // Use updated demands to run motors
                if (_state->armed && !_state->failsafe && !_olc->inactive()) {
                    _actuator->run(demands);
                }
             }

    };  // ClosedLoopTask

} // namespace hf
