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
#include <msp/serializer.hpp>

class ClosedLoopControl {

    private:

        SnnHelper _helper;

    public:

        void init()
        {
            _helper.init();
        }

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
            (void)serializer;
        }
};
