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

    void onRequest() 
    {
        static uint32_t count;
        Wire1.printf(" Packets: %u", count++);
    }

    class CommsTask {

        public:

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

                    /*
                       const float vals[10] = {
                       state.dx, state.dy, state.z, state.dz, state.phi, state.dphi,
                       state.theta, state.dtheta, state.psi, state.dpsi
                       };

                       _msp.serializeFloats(Msp::MSG_STATE, vals, 10);

                       const int16_t vals[10] = { 99, 100, 101, 102, 103, 104, 105, 106, 107, 108 };

                       _msp.serializeShorts(Msp::MSG_STATE, vals, 10);

                       Serial3.write(_msp.payload, _msp.payloadSize);

                    //while (_msp.available()) {
                    //    Serial3.write(_msp.read());
                    //}
                     */
                }
            }

        private:

            static const uint8_t I2C_DEV_ADDR = 0x55;

            Timer _timer;

            Msp _msp;
    };

}
