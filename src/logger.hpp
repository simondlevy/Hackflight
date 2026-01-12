/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


#include <comms.hpp>
#include <msp/__messages__.h>
#include <msp/serializer.hpp>
#include <timer.hpp>

class Logger {

    public:

        static void run(
                const uint32_t msec_curr,
                const vehicleState_t & state)
        {
            static Timer _timer;

            if (_timer.ready(FREQ_HZ)) {

                MspSerializer serializer = {};

                const float statevals[10] = { state.dx, state.dy, state.z,
                    state.dz, state.phi, state.dphi, state.theta,
                    state.dtheta, state.psi, state.dpsi 
                };

                serializer.serializeFloats(MSP_STATE, statevals, 10);

                sendPayload(serializer);
            }
        }

    private:

        static constexpr float FREQ_HZ = 100;

        static void sendPayload(const MspSerializer & serializer) {
            for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                Comms::write_byte(serializer.payload[k]);
            }
        }
};
