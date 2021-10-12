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

            SerialTask serialTask;

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
                _state.armed = false;
            }

            void update(
                    uint32_t time_usec,
                    float * motorvals,
                    bool * led,
                    bool * serialReady)
            {
                HackflightPure::update(time_usec, motorvals);

                *serialReady = serialTask.ready(time_usec);

                checkSafety(&_state, motorvals);

                *led = time_usec < 2000000 ? (time_usec / 50000) % 2 == 0 : _state.armed;
            }

            void serialParse(uint8_t byte, float * motorvals)
            {
                serialTask.parse(byte, _state.state, _mixer, motorvals);
            }

            uint8_t serialAvailable(void)
            {
                return serialTask.available();
            }

            uint8_t serialRead(void)
            {
                return serialTask.read();
            }

    }; // class HackflightFull

} // namespace
