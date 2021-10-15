/*
   Hackflight class supporting safety checks and serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pure.hpp"
#include "HF_safety.hpp"
#include "HF_serial.hpp"
#include "HF_debugger.hpp"

#include "stream_receiver.h"

namespace hf {

    class HackflightFull : public HackflightPure {

        private:

            SerialComms _serial;
            Safety _safety;

            void checkSafety(state_t & state, bool receiverInArmedState, bool receiverThrottleIsDown,  motors_t & motors)
            {
                bool failsafe = stream_receiverLostSignal && _safety.armed;

                bool disarm = _safety.armed && !receiverInArmedState;

                bool throttleDown = _safety.armed && receiverThrottleIsDown;

                bool running = _safety.armed && !receiverThrottleIsDown;

                // Arm after lots of safety checks
                bool arm = !_safety.armed
                        && !_safety.failsafe 
                        && Safety::safeToArm(state)
                        && receiverThrottleIsDown
                        && receiverInArmedState;

                bool cut = failsafe || disarm || throttleDown; 

                motors.ready = failsafe || disarm || arm || throttleDown || running ? true : motors.ready;
                
                motors.values[0] = cut ? 0 : motors.values[0];
                motors.values[1] = cut ? 0 : motors.values[1];
                motors.values[2] = cut ? 0 : motors.values[2];
                motors.values[3] = cut ? 0 : motors.values[3];

                _safety.armed = failsafe || disarm ? false : arm ? true : _safety.armed;

                _safety.failsafe = failsafe ? true : _safety.failsafe;

             }

        public:

            HackflightFull(Mixer * mixer)
                : HackflightPure(mixer)
            {
                _safety.armed = false;
            }

            void update(
                    uint32_t time_usec,
                    float tdmd,
                    float rdmd,
                    float pdmd,
                    float ydmd,
                    bool rxarmed,
                    bool rxtdown,
                    float state_phi,
                    float state_theta,
                    float state_psi,
                    float state_dphi,
                    float state_dtheta,
                    float state_dpsi,
                    motors_t & motors,
                    bool & led)
            {
                HackflightPure::update(
                        time_usec,
                        tdmd,
                        rdmd,
                        pdmd,
                        ydmd,
                        state_phi,
                        state_theta,
                        state_psi,
                        motors);

                Debugger::printf("%+3.3f %+3.3f\n", _state.dphi, state_dphi);

                checkSafety(_state, rxarmed, rxtdown, motors);

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
