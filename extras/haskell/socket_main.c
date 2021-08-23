/*
  Socket code for working with Haskell Copilot version of Hackflight via
  simulator

  Copyright(C) 2021 Simon D.Levy

  MIT License
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <sys/time.h>

#include "udp_sockets.h"
#include "hackflight.h"

static const char * HOST = "127.0.0.1";
static const uint16_t MOTOR_PORT = 5000;
static const uint16_t TELEMETRY_PORT = 5001;

// Avaiable to Copilot
double receiverThrottle = 0;
double receiverRoll = 0;
double receiverPitch = 0;
double receiverYaw = 0;

double gyrometerX = 0;
double gyrometerY = 0;
double gyrometerZ = 0;

double eulerX = 0;
double eulerY = 0;
double eulerZ = 0;

double altimeterZ = 0;
double altimeterDz = 0;

double flowX = 0;
double flowY = 0;

double time = 0;

// Shared by main() and runMotors()
static udp_socket_t motor_client_socket; 

// Called by Copilot
void runMotors(double m1, double m2, double m3, double m4)
{
    double values[4] = {m1, m2, m3, m4};
    udp_send_data(motor_client_socket, values, 4*sizeof(double));
}

int main (int argc, char *argv[])
{
    udp_client_socket_init(&motor_client_socket, HOST, MOTOR_PORT, 0);

    udp_socket_t telemetry_server_socket = {};
    udp_server_socket_init(&telemetry_server_socket, TELEMETRY_PORT, 0);

    printf("Hit the start button ...\n");
    fflush(stdout);

    double start_time = 0;

    while (true) {

        double telemetry_data[17] = {};

        if (udp_receive_data(
                    telemetry_server_socket,
                    telemetry_data,
                    sizeof(telemetry_data))) {

            // Simulate altimeter/variometer
            altimeterZ = telemetry_data[5];
            altimeterDz = telemetry_data[6];

            // XXX Get Euler angles directly for now; really want to simulate
            // quaternion
            eulerX = telemetry_data[7];
            eulerY = telemetry_data[9];
            eulerZ = telemetry_data[11];

            // Simulate gyrometer
            gyrometerX = telemetry_data[8];
            gyrometerY = telemetry_data[10];
            gyrometerZ = telemetry_data[12];

            // Simulate optical flow by rotating earth-frame X,Y velocity into
            // body frame.  To keep it simple we ignore pitch and roll.
            double dx = telemetry_data[2];
            double dy = telemetry_data[4];
            double psi = telemetry_data[11];
            double cp = cos(psi);
            double sp = sin(psi);
            flowX = cp * dx + sp * dy;
            flowY = cp * dy - sp * dx;

            // Simulate receiver
            receiverThrottle = telemetry_data[13];
            receiverRoll = telemetry_data[14];
            receiverPitch = telemetry_data[15];
            receiverYaw = telemetry_data[16];

            // Get current time, starting with 0
            struct timeval tv = {};
            gettimeofday(&tv, NULL);
            double curr_time = (double)(tv.tv_sec + tv.tv_usec/1e6);
            if (start_time == 0) {
                start_time = curr_time;
            }
            time = curr_time - start_time;

            udp_set_timeout(telemetry_server_socket, 100);

            // Call Copilot
            step();
        }

        else {

            exit(0);
        }
    }

    return 0;
}
