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

#include <math.h>

#include <clock.hpp>
#include <control/pids/altitude.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <datatypes.h>
#include <num.hpp>
#include <vehicles/diyquad.hpp>

#include <posix-utils/socket.hpp>

#include "framework_utils.hpp"

static const uint16_t VIZ_PORT = 8100;
static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

static const char * NETWORK_FILENAME =
"/home/levys/Desktop/2025-diff-network/diff_network.txt";

static const double SPIKE_TIME_MAX = 1000;

static double cap(const double val)
{
    return val > +1 ? +1 : val < -1 ? -1 : val;
}

static double value_to_spike_time(const double val)
{
    return FrameworkUtils::get_spike_time(cap(val), SPIKE_TIME_MAX);
}

static float runSnn(float demand, float actual)
{
    static constexpr float KP = 25;

    static Network * _net;
    static Processor * _proc;
    static ServerSocket _serverSocket;

    // Initialize the first time around
    if (!_net) {

        // Load the network
        _net = FrameworkUtils::load(NETWORK_FILENAME, &_proc);

        // Listen for and accept connections from vizualization client
        _serverSocket.open(VIZ_PORT);
#ifdef _VIZ
        _serverSocket.acceptClient();
#endif
    }

    // Turn the demand and climb-rate into spikes
    const double spike_time_1 = value_to_spike_time(demand);
    const double spike_time_2 = value_to_spike_time(actual);

    // Apply the spikes to the network
    vector <Spike> spikes_array = {};
    FrameworkUtils::apply_spike(_net, _proc, 0, spike_time_1, spikes_array);
    FrameworkUtils::apply_spike(_net, _proc, 1, spike_time_2, spikes_array);
    FrameworkUtils::apply_spike(_net, _proc, 2, 0, spikes_array);

    // Run the network
    const double sim_time = 3 * SPIKE_TIME_MAX + 2;
    _proc->run(sim_time);
    spikes_array.clear();

    // Get the output network's firing time
    const double out = _proc->output_vectors()[0][0];
    const double time = out == SPIKE_TIME_MAX + 1 ? -2 : out;

    // Convert the firing time to a difference in [-2,+2]
    const double diff = (time-SPIKE_TIME_MAX)*2 / SPIKE_TIME_MAX - 2;

    // Periodically send the spike counts to the visualizer
    static uint32_t _vizcount;
    if (_vizcount++ == VIZ_SEND_PERIOD) {
        const string msg = FrameworkUtils::make_viz_message(_net, _proc);
        printf("%s\n", msg.c_str());
#ifdef _VIZ
#endif
        _vizcount = 0;
    }

    // Convert the difference into a thrust, constrained by motor limits
    return Num::fconstrain(KP * diff * THRUST_SCALE + THRUST_BASE,
            THRUST_MIN, THRUST_MAX); 
}

static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    (void)landingAltitudeMeters;

    const auto climbrate = AltitudeController::run(hovering,
            dt, vehicleState.z, openLoopDemands.thrust);

    demands.thrust = runSnn(climbrate, vehicleState.dz);

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
}
