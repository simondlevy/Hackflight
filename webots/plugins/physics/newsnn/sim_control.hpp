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

#include <framework.hpp>
#include <risp.hpp>

static const uint16_t VIZ_PORT = 8100;
static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

static const double SPIKE_TIME_MAX = 1000;

static double cap(const double val)
{
    return val > +1 ? +1 : val < -1 ? -1 : val;
}

static double get_spike_time(const double inp, const double max)
{
    return round(max * (1 - inp) / 2);
}

static double value_to_spike_time(const double val)
{
    return get_spike_time(cap(val), SPIKE_TIME_MAX);
}

static std::string make_viz_message(
        const Network & net,
        const std::vector<int> counts)
{
    const size_t n = counts.size();

    std::string msg = "{\"Event Counts\":[";

    for (size_t i=0; i<n; ++i) {
        char tmp[100] = {};
        const int count = counts[i];
        sprintf(tmp, "%d%s", count, i==n-1 ? "]"  : ", ");
        msg += tmp;
    }

    msg += ", \"Neuron Alias\":[";

    for (size_t i=0; i<n; ++i) {
        char tmp[100] = {};
        sprintf(tmp, "%d%s", 
                net.sorted_node_vector[i]->id, i==n-1 ? "]" : ", ");
        msg += tmp;
    }

    return msg + "}\n"; 
}

static void apply_spike(
        Network & net,
        Processor *p,
        const int spike_id,
        const double spike_time,
        const double spike_val=1,
        const bool normalize=true) 
{
    p->apply_spike(Spike(net.get_node(spike_id)->input_id,
                spike_time, spike_val), normalize);

}

static float runSnn(float demand, float actual)
{
    static constexpr float KP = 25;

    static bool _initialized;
    static Network  _net;
    static risp::Processor _proc;
    static ServerSocket _serverSocket;

    // Initialize the first time around
    if (!_initialized) {

        // Load the network
        _proc.load_network(&_net);

        // Listen for and accept connections from vizualization client
        _serverSocket.open(VIZ_PORT);
        _serverSocket.acceptClient();

        _initialized = true;
    }

    // Turn the demand and climb-rate into spikes
    const double spike_time_1 = value_to_spike_time(demand);
    const double spike_time_2 = value_to_spike_time(actual);

    // Apply the spikes to the network
    apply_spike(_net, &_proc, 0, spike_time_1);
    apply_spike(_net, &_proc, 1, spike_time_2);
    apply_spike(_net, &_proc, 2, 0);

    // Run the network
    const double sim_time = 3 * SPIKE_TIME_MAX + 2;
    _proc.run(sim_time);

    // Get the output node's firing time
    const double out = _proc.get_output_fire_times()[0];
    const double time = out == SPIKE_TIME_MAX + 1 ? -2 : out;

    // Convert the firing time to a difference in [-2,+2]
    const double diff = (time-SPIKE_TIME_MAX)*2 / SPIKE_TIME_MAX - 2;

    // Periodically send the spike counts to the visualizer
    static uint32_t _vizcount;
    const double I_SCALE = 0.05;
    const double D_SCALE = 0.25;
    const double O_BIAS = 1000;
    const double O_SCALE = 0.025;
    const double S_BIAS = 800;
    const double S_SCALE = 0.125;
    if (_vizcount++ == VIZ_SEND_PERIOD) {
        const auto tmp = _proc.get_neuron_counts();
        const std::vector<int> counts = {
            (int)(spike_time_1 * I_SCALE),
            (int)(spike_time_2 * I_SCALE),
            1,
            (int)(tmp[3] * D_SCALE),
            (int)(tmp[4] * D_SCALE),
            (int)((out - O_BIAS) * O_SCALE),
            (int)((tmp[6] - S_BIAS) * S_SCALE)
        };

        const std::string msg = make_viz_message(_net, counts);
        _serverSocket.sendData((uint8_t *)msg.c_str(), msg.length());
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
