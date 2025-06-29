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

#include <control/partial.hpp>
#include <tennlab/differencer.hpp>

static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    static DifferenceNetwork _network;

    static bool _initialized;

    // Initialize the first time around
    if (!_initialized) {

        // true = visualize
        _network.init(true);

        _initialized = true;
    }

    const float zerror = _network.run(openLoopDemands.thrust, vehicleState.z);

    runControlWithZError(
            hovering,
            dt,
            landingAltitudeMeters,
            vehicleState,
            zerror,
            openLoopDemands.roll,
            openLoopDemands.pitch,
            openLoopDemands.yaw,
            demands);
}
