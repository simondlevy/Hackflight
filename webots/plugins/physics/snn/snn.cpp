/* 
 * Custom physics plugin for Hackflight simulator using Spiking Neural Net
 * controllers
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

#include "../support.hpp"

// TeNNLab framework
#include <levy_snn_util.hpp>

static const float VIZ_FREQ = 100; // Hz
static const uint16_t VIZ_PORT = 8100;

static const float PITCH_ROLL_PRE_DIVISOR = 10; // deg

static const float YAW_DIVISOR  = 26;
static const float YAW_OFFSET = 0.955;

static const float CLIMBRATE_DIVISOR  = 3;
static const float CLIMBRATE_OFFSET = 8.165;

static const float CASCADE_DIVISOR  = 15;
static const float CASCADE_OFFSET = 0.936;
static const float CASCADE_POST_SCALE = 120;

static const char * NETWORK = "networks/difference_risp_train.txt";
static const char * NETWORK3 = "networks/difference3_risp.txt";

static const float TAKEOFF_TIME = 2; // sec
static const float MOTOR_TAKEOFF = 75; // rad/sec
//static const float MOTOR_HOVER = 74.375; // rad/sec

static const float YAW_SCALE = 160; // deg/s

static SNN * climbRateSnn;
static SNN * yawRateSnn;
static SNN * cascadeSnn;

// Choose from one of the three networks above to visualize
static auto vizSnn = &climbRateSnn;

static hf::state_t estimateState()
{
    return hf::state_t {
        dynamics._x1,
            dynamics._x2 * cos(dynamics._x11) -
                dynamics._x4 * sin(dynamics._x11),
        dynamics._x3,
        -(dynamics._x2 * sin(dynamics._x11) +
                    dynamics._x4 * cos(dynamics._x11)),
        dynamics._x5,
        dynamics._x6,
            hf::Utils::RAD2DEG* dynamics._x7,
            hf::Utils::RAD2DEG* dynamics._x8,
            hf::Utils::RAD2DEG* dynamics._x9,
            hf::Utils::RAD2DEG* dynamics._x10,
            hf::Utils::RAD2DEG* dynamics._x11,
            hf::Utils::RAD2DEG* dynamics._x12,
    };
}


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
        printf("%d", counts[0]);
        //printf("%f,%f\n", setpoint - actual, action);
    }

    return action;
}

static float runCascadeSnn(
        SNN * snn, const float inp1, const float inp2, const float inp3)
{
    vector<double> observations = { inp1, inp2, inp3 };

    vector <int> counts = {};

    snn->step(observations, counts);

    return counts[0] / CASCADE_DIVISOR - CASCADE_OFFSET;
}


// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    hf::siminfo_t siminfo = {};

    if (!getSimInfo(siminfo)) {
        return;
    }

    // Run control in middle loop
    for (uint32_t j=0; j <outerLoopCount(siminfo);  ++j) {

        // Start with open-loop demands
        hf::demands_t demands = {
            siminfo.demands.thrust,
            siminfo.demands.roll,
            siminfo.demands.pitch,
            siminfo.demands.yaw
        };

        // Get current state from dynamics model
        const auto state = estimateState();

        // Run PID controllers to get final demands

        const auto thrustFromSnn = runSnn(
                climbRateSnn, demands.thrust, state.dz,
                CLIMBRATE_DIVISOR, CLIMBRATE_OFFSET);

        demands.yaw = runSnn(
                yawRateSnn,
                demands.yaw / YAW_SCALE,
                state.dpsi / YAW_SCALE,
                YAW_DIVISOR, YAW_OFFSET);

        const auto phi = state.phi / PITCH_ROLL_PRE_DIVISOR;
        
        const auto snn_diff = runCascadeSnn(
                cascadeSnn, demands.roll, state.dy, phi);

        demands.roll = 60 * snn_diff;
        demands.roll = 0.0125 * (demands.roll - state.dphi);

        demands.pitch = 6 * (10 * (demands.pitch - state.dx) - state.theta);
        demands.pitch = 0.0125 * (demands.pitch - state.dtheta);

        // Once takeoff has been requested, we compute time using the 
        // deltaT from the middle (control) loop
        static uint32_t _count;
        _count = siminfo.requested_takeoff ? (_count + 1) : 0;
        const float time = _count * pidDt();

        // Ignore thrust demand until airborne, based on time from launch
        demands.thrust =
            time > TAKEOFF_TIME ? 
            thrustFromSnn + MOTOR_HOVER:
            siminfo.requested_takeoff ? 
            MOTOR_TAKEOFF :
            0;

        // Update dynamics in innermost loop
        updateDynamics(demands);

        static uint32_t _vizcount;
        if (_vizcount++ % 100 == 0) {
            // Send spikes to visualizer
            (*vizSnn)->send_counts_to_visualizer();
        }
    }

    // Set pose in outermost loop
    setPose(dynamics);

}

// Called by webots_physics_init()
void setup_controllers()
{
    try {

        climbRateSnn = makeSnn(NETWORK);
        yawRateSnn = makeSnn(NETWORK);
        cascadeSnn = makeSnn(NETWORK3);

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    (*vizSnn)->serve_visualizer(VIZ_PORT);
}
