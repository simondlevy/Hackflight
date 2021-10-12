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

            void checkSafety(State & state, motors_t & motors)
            {
                // Safety
                static bool _safeToArm;

                // Sync failsafe to open-loop-controller
                if (stream_receiverLostSignal && state.armed) {
                    cutMotors(motors);
                    state.armed = false;
                    state.failsafe = true;
                    return;
                }

                // Disarm
                if (state.armed && !_receiver->inArmedState()) {
                    state.armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!_safeToArm) {
                    _safeToArm = !_receiver->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !state.armed
                    && !state.failsafe 
                    && state.safeToArm()
                    && _receiver->inactive()
                    && _receiver->inArmedState()
                    ) {
                    state.armed = true;
                }

                // Cut motors on inactivity
                if (state.armed && _receiver->inactive()) {
                    cutMotors(motors);
                }

            } // checkSafety

            static void cutMotors(motors_t & motors)
            {
                motors.values[0] = 0;
                motors.values[1] = 0;
                motors.values[2] = 0;
                motors.values[3] = 0;
            }

        public:

            HackflightFull(Receiver * receiver, Mixer * mixer)
                : HackflightPure(receiver, mixer)
            {
                _state.armed = false;
            }

            void update(
                    uint32_t time_usec,
                    motors_t & motors,
                    bool & led,
                    bool & serialReady)
            {
                HackflightPure::update(time_usec, motors);

                serialReady = serialTask.ready(time_usec);

                checkSafety(_state, motors);

                led = time_usec < 2000000 ? (time_usec / 50000) % 2 == 0 : _state.armed;
            }

            void serialParse(uint8_t byte, motors_t & motors)
            {
                serialTask.parse(byte, _state.state, _mixer, motors);
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
