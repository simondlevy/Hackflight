/*
   Hackflight communications task

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <Wire.h>

#include <hackflight.hpp>
#include <timer.hpp>
#include <msp.hpp>

namespace hf {

    static uint8_t msg[100];
    static uint8_t msgsize;

    class CommsTask {

        private:

            static void onRequest() 
            {
                Wire1.write(msg, msgsize);
            }

            Timer _timer;

            Msp _msp;

        public:

            static const uint8_t I2C_DEV_ADDR = 0x55;

            void begin()
            {
                Wire1.onRequest(onRequest);

                // Join the I^2C bus as a peripheral device
                Wire1.begin(I2C_DEV_ADDR);
            }

            void run(
                    const state_t & state,
                    const uint32_t usec_curr,
                    const float freq_hz)
            {
                if (_timer.isReady(usec_curr, freq_hz)) {

                    const float vals[10] = {
                        state.dx, state.dy, state.z, state.dz, state.phi, state.dphi,
                        state.theta, state.dtheta, state.psi, state.dpsi
                    };

                    _msp.serializeFloats(Msp::MSG_STATE, vals, 10);

                    memcpy(msg, _msp.payload, _msp.payloadSize);

                    msgsize = _msp.payloadSize;
                }
            }
    };

}
