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

            SerialComms _serial;
            Safety _safety;

            void checkSafety(state_t & state, motors_t & motors)
            {
                bool failsafe = stream_receiverLostSignal && _safety.armed;

                bool disarm = _safety.armed && !_receiver->inArmedState();

                bool throttleDown = _safety.armed && _receiver->inactive();

                bool running = _safety.armed && !_receiver->inactive();

                // Arm after lots of safety checks
                bool arm = !_safety.armed
                        && !_safety.failsafe 
                        && Safety::safeToArm(state)
                        && _receiver->inactive()
                        && _receiver->inArmedState();

                bool cut = failsafe || disarm || throttleDown; 

                motors.ready = failsafe || disarm || arm || throttleDown || running ? true : motors.ready;
                
                motors.values[0] = cut ? 0 : motors.values[0];
                motors.values[1] = cut ? 0 : motors.values[1];
                motors.values[2] = cut ? 0 : motors.values[2];
                motors.values[3] = cut ? 0 : motors.values[3];

                _safety.armed = failsafe || disarm ? false : arm ? true : _safety.armed;

                _safety.failsafe = failsafe ? true : _safety.failsafe;

             } // checkSafety

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

                _serial.parse(_state, _mixer, motors);
            }

            uint8_t serialAvailable(void)
            {
                return _serial.available();
            }

            uint8_t serialRead(void)
            {
                return _serial.read();
            }

    }; // class HackflightFull

} // namespace
