/*
   Timer task for PID controllers

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "timertask.hpp"

#include <RFT_actuator.hpp>

namespace hf {

    class PidTask : public TimerTask {

        friend class Hackflight;

        private:

            // For now, we keep all PID timer tasks the same.  At some point it might be useful to 
            // investigate, e.g., faster updates for Rate PID than for Level PID.
            static constexpr float FREQ = 300;

            // PID controllers
            PidController * _pid_controllers[256] = {};
            uint8_t _pid_controller_count = 0;

            // Other stuff we need
            Receiver * _receiver = NULL;
            rft::Actuator * _actuator = NULL;
            State  * _state    = NULL;

        protected:

            PidTask(void)
                : TimerTask(FREQ)
            {
                _pid_controller_count = 0;
            }

            void begin(Board * board, Receiver * receiver, rft::Actuator * actuator, State * state)
            {
                TimerTask::begin(board);

                _receiver = receiver;
                _actuator = actuator;
                _state = state;
            }

            void addPidController(PidController * pidController, uint8_t auxState) 
            {
                pidController->auxState = auxState;

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

                //Debugger::printf("Aux state: %d", auxState);

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PidController * pidController = _pid_controllers[k];

                    // Some PID controllers need to reset their integral when the throttle is down
                    pidController->updateReceiver(_receiver->throttleIsDown());

                    if (pidController->auxState <= auxState) {

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
                    _actuator->run(demands);
                }
             }

    };  // PidTask

} // namespace hf
