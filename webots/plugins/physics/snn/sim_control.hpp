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

#include <difference_network.hpp>

#include <control/partial.hpp>

#include "visualizer.hpp"

static const float MAX_SPIKE_TIME = 1000;

static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    const float zerror = DifferenceNetwork::run(
            openLoopDemands.thrust, vehicleState.z, MAX_SPIKE_TIME);

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

    static bool _initialized;

    static Visualizer _visualizer;

    if (!_initialized) {

        _visualizer.init(MAX_SPIKE_TIME);

        _initialized = true;
    }
}
