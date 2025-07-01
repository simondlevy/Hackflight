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

#include <vector>
#include <string>

#include <posix-utils/server.hpp>
#include <difference_network.hpp>
#include <control/partial.hpp>

static const int VIZ_PORT = 8100;

static const int VIZ_SEND_PERIOD = 50; // ticks

static const float MAX_SPIKE_TIME = 1000;

std::string make_viz_message(const std::vector<int> counts)
{
    const int n = counts.size();

    std::string msg = "{\"Event Counts\":[";

    for (int i=0; i<n; ++i) {
        char tmp[100] = {};
        const int count = counts[i];
        sprintf(tmp, "%d%s", count, i==n-1 ? "]"  : ", ");
        msg += tmp;
    }

    msg += ", \"Neuron Alias\":[";

    for (int i=0; i<n; ++i) {
        char tmp[100] = {};
        sprintf(tmp, "%d%s", i, i==n-1 ? "]" : ", ");
        msg += tmp;
    }

    return msg + "}\n"; 
}

static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    static bool _initialized;
    static DifferenceNetwork _net;
    static ServerSocket _spikeServer;

    if (!_initialized) {

        _net.init(MAX_SPIKE_TIME);

        _spikeServer.open(VIZ_PORT);
        _spikeServer.acceptClient();

        _initialized = true;
    }    

    const float zerror = _net.run(openLoopDemands.thrust, vehicleState.z);

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

    static int _tick;

    if (_tick++ == VIZ_SEND_PERIOD) {

        const std::vector<int> counts = {10, 10, 10, 10, 10, 10, 10};

        const std::string msg = make_viz_message(counts);

        _spikeServer.sendData((uint8_t *)msg.c_str(), msg.length());

        _tick = 0;
    }

    //_visualizer.send_spikes(10, 10, 10, 10, 10, 10);
}
