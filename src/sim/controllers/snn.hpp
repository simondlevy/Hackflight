/* 
 * Hackflight simulator using Spiking Neural Net * controllers
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// Hackflight
#include <sim/vehicles/diyquad.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>

// TeNNLab framework
#include <snn_util.hpp>

static const float VIZ_FREQ = 100; // Hz
static const uint16_t VIZ_PORT = 8100;

static const float THROTTLE_DOWN = 0.06;
static const float PITCH_ROLL_POST_SCALE = 50;

static const float YAW_DIVISOR  = 26;
static const float YAW_OFFSET = 0.955;

static const float CLIMBRATE_DIVISOR  = 3;
static const float CLIMBRATE_OFFSET = 8.165;

static const char * NETWORK = ("/home/levys@ad.wlu.edu/Desktop/framework/" 
        "networks/difference_risp_train.txt");

static const float TAKEOFF_TIME = 2; // sec
static const float MOTOR_TAKEOFF = 75; // rad/sec

static const float YAW_SCALE = 160; // deg/s

static SNN * climbRateSnn;
static SNN * yawRateSnn;

// Choose from one of the three networks above to visualize
static auto vizSnn = &climbRateSnn;

static SNN * makeSnn(const char * filename)
{
    return new SNN(filename, "risp");
}

static double runSnn(
        SNN * snn,
        const float setpoint,
        const float actual,
        const float divisor,
        const float offset,
        const bool debug=false)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    const double action = counts[0] / divisor - offset;

    if (debug) {
        //printf("%d", counts[0]);
        //printf("%f,%f\n", setpoint - actual, action);
    }

    return action;
}

// ---------------------------------------------------------------------------

namespace hf {

    static PitchRollAnglePid _pitchRollAnglePid;
    static PitchRollRatePid _pitchRollRatePid;

    // Called by webots_physics_init()
    void setup_controllers()
    {
        try {

            climbRateSnn = makeSnn(NETWORK);
            yawRateSnn = makeSnn(NETWORK);

        } catch (const SRE &e) {
            fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
            exit(1);
        }

        (*vizSnn)->serve_visualizer(VIZ_PORT);
    }

    // Called by webots_physics_step()
    demands_t run_controllers(
            const float pid_dt,
            const siminfo_t & siminfo,
            const state_t & state)
    {
        const auto open_loop_demands = siminfo.demands;

        // Throttle-down should reset pids
        const auto resetPids = open_loop_demands.thrust < THROTTLE_DOWN;

        // Start with open-loop demands
        demands_t demands = {
            open_loop_demands.thrust,
            open_loop_demands.roll,
            open_loop_demands.pitch,
            open_loop_demands.yaw
        };

        PositionPid::run(state, demands);

        _pitchRollAnglePid.run(pid_dt, resetPids, state, demands);

        _pitchRollRatePid.run(pid_dt, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        // Run PID controllers to get final demands

        const auto thrustFromSnn = runSnn(
                climbRateSnn, demands.thrust, state.dz,
                CLIMBRATE_DIVISOR, CLIMBRATE_OFFSET);

        demands.yaw = runSnn(
                yawRateSnn,
                demands.yaw / YAW_SCALE,
                state.dpsi / YAW_SCALE,
                YAW_DIVISOR, YAW_OFFSET);

        // Once takeoff has been requested, we compute time using the 
        // deltaT from the middle (control) loop
        static uint32_t _count;
        _count = siminfo.requested_takeoff ? (_count + 1) : 0;
        const float time = _count * pid_dt;

        // Ignore thrust demand until airborne, based on time from launch
        demands.thrust =
            time > TAKEOFF_TIME ? 
            thrustFromSnn + MOTOR_HOVER:
            siminfo.requested_takeoff ? 
            MOTOR_TAKEOFF :
            0;

        static uint32_t _vizcount;
        if (_vizcount++ % 100 == 0) {
            // Send spikes to visualizer
            climbRateSnn->send_counts_to_visualizer();
        }

        return demands;
    }
}
