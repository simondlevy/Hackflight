/*
  C++ client for MulticopterSim

  Copyright(C) 2023 Simon D.Levy

  This file is part of SimFlightControl.

  SimFlightControl is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option)
  any later version.

  SimFlightControl is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  SimFlightControl. If not, see <https://www.gnu.org/licenses/>.

 */

#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

#include <pid.h>
#include <pids/angle.h>
#include <pids/setpoints/althold.h>
#include <mixers/fixedpitch/quadxbf.h>

#include "../sockets/UdpClientSocket.hpp"
#include "../sockets/UdpServerSocket.hpp"

// Comms
static const char * HOST = "127.0.0.1"; // localhost
static uint16_t  MOTOR_PORT = 5000;
static uint16_t  TELEM_PORT = 5001;

static VehicleState state_from_telemetry(const double telemetry[])
{
    return VehicleState( 
            telemetry[1],   // x
            telemetry[2],   // dx
            telemetry[3],   // y
            telemetry[4],   // dy
            telemetry[5],   // z  
            telemetry[6],   // dz 
            telemetry[7],   // phi
            telemetry[8],   // dphi
            -telemetry[9],  // theta  Make nose-down positive
            -telemetry[10], // dtheta  for PID controller
            telemetry[11],  // psi
            telemetry[12]   // dpsi
            );
}

static Demands demands_from_telemetry(const double telemetry[])
{
    return Demands(
            (float)telemetry[13], 
            (float)telemetry[14],
            (float)telemetry[15],
            (float)telemetry[16]
            );
}

int main(int argc, char ** argv)
{
    // Create sockets for telemetry in, motors out
    UdpServerSocket telemServer = UdpServerSocket(TELEM_PORT);
    UdpClientSocket motorClient = UdpClientSocket(HOST, MOTOR_PORT);

    // Create Hackflight objects

    static AnglePidController anglePid = 
        AnglePidController(
                10, // K_rate_p
                10, // K_rate_i
                1,  // K_rate_d
                0,  // K_rate_f
                4); // K_level_p

    static AltHoldPidController altHoldPid;

    static Mixer mixer = QuadXbfMixer::make();

    printf("Hit the Play button ... ");
    fflush(stdout);

    std::vector<PidController *> pids = { &anglePid, &altHoldPid };

    // Loop forever, waiting for clients
    while (true) {

        // Get incoming telemetry values
        double telemetry[17] = {};
        telemServer.receiveData(telemetry, sizeof(telemetry));

        // Sim sends negative time value on halt
        double time = telemetry[0];
        if (time < 0) {
            break;
        }

        // Convert simulator time to microseconds
        const auto usec = (uint32_t)(time * 1e6);

        // Build vehicle state 
        auto vstate = state_from_telemetry(telemetry);

        // Build stick demands
        auto demands = demands_from_telemetry(telemetry);

        // Reset PID controllers on zero throttle
        auto pidReset = demands.throttle < .05;

        // Run stick demands through PID controllers to get final demands
        PidController::run(pids, demands, vstate, usec, pidReset);

        // Run final demands through mixer to get motor values
        float mvals[4] = {};
        mixer.getMotors(demands, mvals);

        // Send back motor values
        motorClient.sendData(mvals, sizeof(mvals));

    } // while (true)

    return 0;
}
