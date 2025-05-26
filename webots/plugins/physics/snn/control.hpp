/* 
 * Custom physics plugin for Hackflight simulator using ground-truth state 
 * and Spiking Neural Net controllers
 *
 *  Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
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

#include <stdlib.h>

#include <posix-utils/socket.hpp>

#include <control/pids/altitude.hpp>
#include <control/pids/climbrate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>

#include <vehicles/diyquad.hpp>

#include <snn_util.hpp>

static const uint16_t VIZ_PORT = 8100;
static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

static const float CLIMBRATE_SCALE  = 500;
static const float CLIMBRATE_OFFSET = 26000;

static const char * NETWORK = "difference_risp_train";

static SNN * climbRateSnn;

//static auto vizSnn = &climbRateSnn;

static SNN * makeSnn()
{
    char filename[100] = {};

    sprintf(filename, "%s/Desktop/framework/networks/%s.txt",
            getenv("HOME"), NETWORK);

    return new SNN(filename, "risp");
}

static double runSnn(
        SNN * snn,
        const float setpoint,
        const float actual,
        const float scale,
        const float offset,
        const bool debug=false)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    const double action = counts[0] * scale + offset;

    if (debug) {
        printf("%d", counts[0]);
        printf("%f,%f\n", setpoint - actual, action);
    }

    return action;
}

//static ServerSocket serverSocket;

void runClosedLoopControl(
        const float dt,
        const bool inHoverMode,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    if (!climbRateSnn) {

        try {

            climbRateSnn = makeSnn();

        } catch (const SRE &e) {
            fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
            exit(1);
        }

        // Listen for and accept connections from vizualization client
        //serverSocket.open(VIZ_PORT);
        //serverSocket.acceptClient();
    }

    const auto climbrate = AltitudeController::run(inHoverMode,
            dt, vehicleState.z, openLoopDemands.thrust);

    // Run SNN in hover mode only; otherwise, use standard PID controller
    demands.thrust = inHoverMode ?

        runSnn(climbRateSnn, climbrate, vehicleState.dz,
                CLIMBRATE_SCALE, CLIMBRATE_OFFSET, true) :

            ClimbRateController::run(
                    inHoverMode,
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
            inHoverMode ? openLoopDemands.pitch : 0,
            inHoverMode ? openLoopDemands.roll : 0,
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

    /*
    static uint32_t _vizcount;
    if (_vizcount++ % VIZ_SEND_PERIOD == 0) {
        // Send spikes to visualizer
        uint8_t counts[256] = {};
        const auto ncounts = (*vizSnn)->get_counts(counts);
        serverSocket.sendData(counts, ncounts);
    }*/
}
