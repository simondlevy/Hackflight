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
#include <stdio.h>

#include <clock.hpp>
#include <control/pids/altitude.hpp>
#include <control/pids/climbrate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <datatypes.h>
#include <num.hpp>

#include "framework_utils.hpp"

static const char * NETWORK_FILENAME =
"/home/levys/Desktop/diffnet/difference_risp_plank.txt";

static const double MAX = 1000;

static Network * net;

static Processor * proc;

static float runSnn(const float dz, const float demand)
{
    static constexpr float KP = 25;

    if (!net) {
        net = FrameworkUtils::load(NETWORK_FILENAME, &proc);
    }

    const double spike_time_1 = FrameworkUtils::get_spike_time(demand, MAX);

    const double spike_time_2 = FrameworkUtils::get_spike_time(dz, MAX);

    vector <Spike> spikes_array = {};

    FrameworkUtils::apply_spike(net, proc, 0, spike_time_1,
            spikes_array);
    FrameworkUtils::apply_spike(net, proc, 1, spike_time_2,
            spikes_array);
    FrameworkUtils::apply_spike(net, proc, 2, 0,
            spikes_array);

    const auto sim_time = 3 * MAX + 2;

    proc->run(sim_time);

    spikes_array.clear();

    const auto out = proc->output_vectors()[0][0];

    const auto time = out == MAX + 1 ? -2 : out;

    const auto diff = (time-(MAX))*4/(2*MAX)-2;

    const auto error = demand - dz;

    printf("%f(%f)\n", diff, error);

    const auto thrust = KP * error;

    return Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
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
    const auto climbrate = AltitudeController::run(hovering,
            dt, vehicleState.z, openLoopDemands.thrust);

    demands.thrust =

        hovering ? 

        runSnn(vehicleState.dz, climbrate)

        : ClimbRateController::run(
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
}
