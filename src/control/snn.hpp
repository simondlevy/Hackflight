/**
 * Copyright 2025 Simon D. Levy
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

#include <control/snn/helper.hpp>
#include <msp/__messages__.h>
#include <msp/serializer.hpp>

class ClosedLoopControl {

    private:

        SnnHelper _helper;

    public:

        void run(
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                const float landingAltitudeMeters,
                demands_t & demands)
        {
            _helper.run(dt, hovering, vehicleState, openLoopDemands,
                    landingAltitudeMeters, demands); 
        }

        void serializeMessage(MspSerializer & serializer)
        {
            const uint8_t counts[16] = {
                (uint8_t)_helper.get_i1_relative_spike_time(),
                (uint8_t)_helper.get_i2_relative_spike_time(),
                (uint8_t)_helper.get_s_relative_spike_time(),
                (uint8_t)_helper.get_d1_relative_spike_time(),
                (uint8_t)_helper.get_d2_relative_spike_time(),
                (uint8_t)_helper.get_s2_relative_spike_time(),
                (uint8_t)_helper.get_o_relative_spike_time(),
                0, 0, 0, 0, 0, 0, 0, 0, 0 };

            serializer.serializeBytes(MSP_SPIKES, counts, 16);
        }
};
