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

#include <max_1000.hpp>

#include <control/partial.hpp>

static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    static bool _initialized;

    // Initialize the first time around
    if (!_initialized) {

        _initialized = true;
    }

    const float zerror = DifferenceNetwork::run(openLoopDemands.thrust, vehicleState.z);
    //const float zerror = openLoopDemands.thrust - vehicleState.z;

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
