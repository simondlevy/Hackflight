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
#include "pidcontroller.hpp"

namespace hf {

    class PidsTask : public TimerTask {

        private:

            static constexpr float FREQ = 300;

            demands_t * _demands = NULL;

            // PID controllers
            PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

        protected:

            // TimerTask overrides -------------------------------------------------------

            virtual void doTask(void) override
            {
                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAux1State();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PidController * pidController = _pid_controllers[k];

                    if (pidController->auxState <= auxState) {

                        pidController->modifyDemands(_state, _demands); 

                        if (pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);
            }

        public:

            PidsTask(void)
                : TimerTask(FREQ)
            {
            }

            void init(Board * board, state_t * state, Mixer * mixer, Receiver * receiver, demands_t * demands) 
            {
                TimerTask::init(board, state, mixer, receiver);

                _demands = demands;
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                pidController->auxState = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void updateReceiverDemands(void)
            {
                for (uint8_t k=0; k<_pid_controller_count; ++k) {
                    _pid_controllers[k]->updateReceiver(_receiver->demands, _receiver->throttleIsDown());
                }
            }

    };  // PidsTask

} // namespace hf
