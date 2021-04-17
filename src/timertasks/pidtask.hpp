/*
   Timer task for PID controllers

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_timertask.hpp>
#include <rft_closedloops/pidcontroller.hpp>

namespace hf {

    class PidTask : public rft::TimerTask {

        friend class Hackflight;

        private:

            // For now, we keep all PID timer tasks the same.  At some point it might be useful to 
            // investigate, e.g., faster updates for Rate PID than for Level PID.
            static constexpr float FREQ = 300;

            // PID controllers
            rft::PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

            // Other stuff we need
            Receiver * _receiver = NULL;
            Mixer * _mixer = NULL;
            State * _state    = NULL;

        // protected:
        public:

            PidTask(void)
                : rft::TimerTask(FREQ)
            {
                _pid_controller_count = 0;
            }

            void begin(rft::Board * board, Receiver * receiver, Mixer * mixer, State * state)
            {
                rft::TimerTask::begin(board);

                _receiver = receiver;
                _mixer = mixer;
                _state = state;
            }

            void addPidController(rft::PidController * pidController, uint8_t auxState) 
            {
                pidController->modeIndex = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

            virtual void doTask(void) override
            {
                // Start with demands from receiver, scaling roll/pitch/yaw by constant
                float demands[4] = {};
                demands[DEMANDS_THROTTLE] = _receiver->demands[DEMANDS_THROTTLE];
                demands[DEMANDS_ROLL]     = _receiver->demands[DEMANDS_ROLL]  * _receiver->_demandScale;
                demands[DEMANDS_PITCH]    = _receiver->demands[DEMANDS_PITCH] * _receiver->_demandScale;
                demands[DEMANDS_YAW]      = _receiver->demands[DEMANDS_YAW]   * _receiver->_demandScale;

                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAux2State();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    rft::PidController * pidController = _pid_controllers[k];

                    // Some PID controllers need to reset their integral when the throttle is down
                    // pidController->updateReceiver(_receiver->throttleIsDown());

                    if (pidController->modeIndex <= auxState) {

                        pidController->modifyDemands(_state, demands); 

                        if (pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);

                // Use updated demands to run motors
                if (_state->armed && !_state->failsafe && !_receiver->throttleIsDown()) {
                    _mixer->run(demands);
                }
             }

    };  // PidTask

} // namespace hf
