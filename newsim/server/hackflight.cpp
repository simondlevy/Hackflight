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

#include <hackflight.hpp>

#include "../sockets/UdpClientSocket.hpp"
#include "../sockets/UdpServerSocket.hpp"

// Comms
static const char * HOST = "127.0.0.1"; // localhost
static uint16_t  MOTOR_PORT = 5000;
static uint16_t  TELEM_PORT = 5001;

static hf::state_t state_from_telemetry(const double telemetry[])
{
    return hf::state_t {
            (float)telemetry[1],   // x
            (float)telemetry[2],   // dx
            (float)telemetry[3],   // y
            (float)telemetry[4],   // dy
            (float)telemetry[5],   // z  
            (float)telemetry[6],   // dz 
            (float)telemetry[7],   // phi
            (float)telemetry[8],   // dphi
            -(float)telemetry[9],  // theta  Make nose-down positive
            -(float)telemetry[10], // dtheta  for PID controller
            (float)telemetry[11],  // psi
            (float)telemetry[12]   // dpsi
            };
}

static hf::demands_t demands_from_telemetry(const double telemetry[])
{
    return hf::demands_t {
            (float)telemetry[13], 
            (float)telemetry[14],
            (float)telemetry[15],
            (float)telemetry[16]
            };
}

int main(int argc, char ** argv)
{
    // Create sockets for telemetry in, motors out
    UdpServerSocket telemServer = UdpServerSocket(TELEM_PORT);
    UdpClientSocket motorClient = UdpClientSocket(HOST, MOTOR_PORT);

    printf("Hit the Play button ... ");
    fflush(stdout);

    auto connected = false;

    // Loop forever, communicating with client
    while (true) {

        // Get incoming telemetry values
        double telemetry[17] = {};
        telemServer.receiveData(telemetry, sizeof(telemetry));

        if (!connected) {
            printf("Client connected\n");
            connected = true;
        }

        // Sim sends negative time value on halt
        double time = telemetry[0];
        if (time < 0) {
            break;
        }

        // Convert simulator time to microseconds
        //const auto usec = (uint32_t)(time * 1e6);

        // Build vehicle state 
        auto state = state_from_telemetry(telemetry);

        // Build stick demands
        const hf::demands_t demands = {0, 0, 0, 0};

        (void)state;
        (void)demands;
        (void)demands_from_telemetry;

        // Run final demands through mixer to get motor values
        float mvals[4] = {};
        //mixer.getMotors(demands, mvals);

        // Send back motor values
        motorClient.sendData(mvals, sizeof(mvals));

    } // while (true)

    return 0;
}
