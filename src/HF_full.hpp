/*
   Hackflight class supporting safety checks and serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pure.hpp"
#include "HF_serial.hpp"

#include "stream_receiver.h"

namespace hf {

    class HackflightFull : public HackflightPure {

        private:

            // Serial tasks
            SerialTask * _serial_tasks[10] = {};
            uint8_t _serial_task_count = 0;

            void checkSafety(State * state, float * motorvals)
            {
                // Safety
                static bool _safeToArm;

                // Sync failsafe to open-loop-controller
                if (stream_receiverLostSignal && state->armed) {
                    cutMotors(motorvals);
                    state->armed = false;
                    state->failsafe = true;
                    return;
                }

                // Disarm
                if (state->armed && !_receiver->inArmedState()) {
                    state->armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!_safeToArm) {
                    _safeToArm = !_receiver->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !state->armed
                    && !state->failsafe 
                    && state->safeToArm()
                    && _receiver->inactive()
                    && _receiver->inArmedState()
                    ) {
                    state->armed = true;
                }

                // Cut motors on inactivity
                if (state->armed && _receiver->inactive()) {
                    cutMotors(motorvals);
                }

            } // checkSafety

            void cutMotors(float * motorvals)
            {
                memset(motorvals, 0, 4*sizeof(float));  // XXX Support other than 4
            }

        public:

            HackflightFull(Receiver * receiver, Mixer * mixer)
                : HackflightPure(receiver, mixer)
            {
                _serial_task_count = 0;
            }

            void begin(void)
            {  
                _state.armed = false;

            } // begin

            void update(uint32_t time_usec, float * motorvals, bool * led)
            {
                HackflightPure::update(time_usec, motorvals);

                checkSafety(&_state, motorvals);

                // Update serial tasks
                for (uint8_t k=0; k<_serial_task_count; ++k) {
                    _serial_tasks[k]->update(time_usec, &_state, _mixer, motorvals);
                }

                *led = time_usec < 2000000 ? (time_usec / 50000) % 2 == 0 : _state.armed;
            }

            void addSerialTask(SerialTask * task)
            {
                _serial_tasks[_serial_task_count++] = task;
            }

    }; // class HackflightFull

} // namespace
