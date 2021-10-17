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

        public:

            HackflightFull(Mixer * mixer)
                : HackflightPure(mixer)
            {
            }

            void update(
                    bool mready,
                    bool mcut,
                    float state_phi,
                    float state_theta,
                    float state_psi,
                    float tdmd,
                    float rdmd,
                    float pdmd,
                    float ydmd,
                    motors_t & motors)
            {
                HackflightPure::update(tdmd, rdmd, pdmd, ydmd, motors);

                motors.ready = mready;
                motors.values[0] = mcut ? 0 : motors.values[0];
                motors.values[1] = mcut ? 0 : motors.values[1];
                motors.values[2] = mcut ? 0 : motors.values[2];
                motors.values[3] = mcut ? 0 : motors.values[3];

                _state.phi = state_phi;
                _state.theta = state_theta;
                _state.psi = state_psi;

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
