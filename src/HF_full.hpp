/*
   Hackflight class supporting safety checks and serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pure.hpp"
#include "HF_serialtask.hpp"

namespace hf {

    class HackflightFull : public HackflightPure {

        private:

            // Safety
            bool _safeToArm = false;

            // Serial tasks
            SerialTask * _serial_tasks[10] = {};
            uint8_t _serial_task_count = 0;

            void checkSafety(State * state, bool * led)
            {
                // Sync failsafe to open-loop-controller
                if (_receiver->lostSignal() && state->armed) {
                    _mixer->cut();
                    state->armed = false;
                    state->failsafe = true;
                    *led = false;
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
                    _mixer->cut();
                }

            } // checkSafety

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
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

                // Initialize the sensors
                startSensors();

                // Start the mixer
                _mixer->begin();

            } // begin

            void update(uint32_t time_usec, bool * led)
            {
                HackflightPure::update(time_usec);

                checkSafety(&_state, led);

                // Update serial tasks
                for (uint8_t k=0; k<_serial_task_count; ++k) {
                    _serial_tasks[k]->update(time_usec, _mixer, &_state);
                }

                *led = time_usec < 2000000 ? (time_usec / 50000) % 2 == 0 : _state.armed;
            }

            void addSerialTask(SerialTask * task)
            {
                _serial_tasks[_serial_task_count++] = task;

                task->init(_receiver, _mixer, &_state);
            }

    }; // class HackflightFull

} // namespace
