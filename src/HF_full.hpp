/*
   Hackflight class supporting safety checks and serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pure.hpp"
#include "HF_safety.hpp"
#include "HF_serial.hpp"

#include "stream_receiver.h"

namespace hf {

    class HackflightFull : public HackflightPure {

        private:

            SerialComms _serialTask;
            Safety _safety;

            void checkSafety(state_t & state, motors_t & motors)
            {
                // Safety
                static bool safeToArm_;

                // Check failsafe
                if (stream_receiverLostSignal && _safety.armed) {
                    cutMotors(motors);
                    motors.ready = true;
                    _safety.armed = false;
                    _safety.failsafe = true;
                    return;
                }

                // Disarm
                if (_safety.armed && !_receiver->inArmedState()) {
                    cutMotors(motors);
                    motors.ready = true;
                    _safety.armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!safeToArm_) {
                    safeToArm_ = !_receiver->inArmedState();
                }

                // Arm after lots of safety checks
                if (safeToArm_
                    && !_safety.armed
                    && !_safety.failsafe 
                    && Safety::safeToArm(state)
                    && _receiver->inactive()
                    && _receiver->inArmedState()
                    ) {
                    _safety.armed = true;
                    motors.ready = true;
                }

                // Cut motors on throttle down
                if (_safety.armed && _receiver->inactive()) {
                    cutMotors(motors);
                    motors.ready = true;
                }

                // Run motors when armed and throttle up
                if (_safety.armed && !_receiver->inactive()) {
                    motors.ready = true;
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
                _safety.armed = false;
            }

            void update( uint32_t time_usec, motors_t & motors, bool & led)
            {
                HackflightPure::update(time_usec, motors);

                checkSafety(_state, motors);

                led = time_usec < 2000000 ? (time_usec / 50000) % 2 == 0 : _safety.armed;
                _serialTask.parse(_state, _mixer, motors);
            }

            void serialParse(motors_t & motors)
            {
            }

            uint8_t serialAvailable(void)
            {
                return _serialTask.available();
            }

            uint8_t serialRead(void)
            {
                return _serialTask.read();
            }

    }; // class HackflightFull

} // namespace
