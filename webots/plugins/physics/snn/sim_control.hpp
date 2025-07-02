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

#include <control/pids/altitude.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <vehicles/diyquad.hpp>

static const int VIZ_PORT = 8100;

static const int VIZ_SEND_PERIOD = 50; // ticks

static const float MAX_SPIKE_TIME = 100;

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

static float runClimbRateController(
        DifferenceNetwork & net,
        const bool hovering,
        const float z0,
        const float dt,
        const float z,
        const float dz,
        const float demand)
{
    static const float KP = 25;
    static const float KI = 15;
    static const float ILIMIT = 5000;

    static float _integral;

    const auto airborne = hovering || (z > z0);

    const auto error = net.run(demand, dz);

    _integral = airborne ? 
        Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

    const auto thrust = KP * error + KI * _integral;

    return airborne ?
        Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
                THRUST_MIN, THRUST_MAX) : 0;
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

    const auto climbrate = AltitudeController::run(hovering,
            dt, vehicleState.z, openLoopDemands.thrust);

    demands.thrust =
        runClimbRateController(
                _net,
                hovering,
                landingAltitudeMeters,
                dt,
                vehicleState.z,
                vehicleState.dz,
                climbrate);

    const auto airborne = demands.thrust > 0;

    const auto yaw = YawAngleController::run(
            airborne, dt, vehicleState.psi, openLoopDemands.yaw);

    demands.yaw =
        YawRateController::run(airborne, dt, vehicleState.dpsi, yaw);

    PositionController::run(
            airborne,
            dt,
            vehicleState.dx, vehicleState.dy, vehicleState.psi,
            hovering ? openLoopDemands.pitch : 0,
            hovering ? openLoopDemands.roll : 0,
            demands.roll, demands.pitch);

    PitchRollAngleController::run(
            airborne,
            dt,
            vehicleState.phi, vehicleState.theta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);

    PitchRollRateController::run(
            airborne,
            dt,
            vehicleState.dphi, vehicleState.dtheta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);

    static int _tick;

    if (_tick++ == VIZ_SEND_PERIOD) {

        const std::vector<int> counts = {
            _net.get_i1_spike_count(),
            _net.get_i2_spike_count(),
            _net.get_s_spike_count(),
            _net.get_d1_spike_count(),
            _net.get_d2_spike_count(),
            _net.get_o_spike_count(),
            _net.get_s2_spike_count()
        };

        const std::string msg = make_viz_message(counts);

        _spikeServer.sendData((uint8_t *)msg.c_str(), msg.length());

        _tick = 0;
    }
}
