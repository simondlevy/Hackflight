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

double quaternionW = 0;
double quaternionX = 0;
double quaternionY = 0;
double quaternionZ = 0;

double gyrometerX = 0;
double gyrometerY = 0;
double gyrometerZ = 0;

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
    printf("%3.3f\n", m1);
    udp_send_data(motor_client_socket, values, 4*sizeof(double));
}

static void simulateAltimeter(double * telemetry_data)
{
    altimeterZ = telemetry_data[5];
    altimeterDz = telemetry_data[6];
}

static void simulateGyrometer(double * telemetry_data)
{
    gyrometerX = telemetry_data[8];
    gyrometerY = telemetry_data[10];
    gyrometerZ = telemetry_data[12];
}

static void simulateQuaternion(double * telemetry_data)
{
    double phi = telemetry_data[7];
    double the = telemetry_data[9];
    double psi = telemetry_data[11];

    // Pre-computation
    float cph = cos(phi/2);
    float cth = cos(the/2);
    float cps = cos(psi/2);
    float sph = sin(phi/2);
    float sth = sin(the/2);
    float sps = sin(psi/2);

    // Conversion
    quaternionW = cph * cth * cps + sph * sth * sps;
    quaternionX = cph * sth * sps - sph * cth * cps;
    quaternionY = -cph * sth * cps - sph * cth * sps;
    quaternionZ = cph * cth * sps - sph * sth * cps;
}

static void simulateOpticalFlow(double * telemetry_data)
{
    // Simulate optical flow by rotating earth-frame X,Y velocity into
    // body frame.  To keep it simple we ignore pitch and roll.
    double dx = telemetry_data[2];
    double dy = telemetry_data[4];
    double psi = telemetry_data[11];
    double cp = cos(psi);
    double sp = sin(psi);
    flowX = cp * dx + sp * dy;
    flowY = cp * dy - sp * dx;
}

static void simulateReceiver(double * telemetry_data)
{
    receiverThrottle = telemetry_data[13];
    receiverRoll = telemetry_data[14];
    receiverPitch = telemetry_data[15];
    receiverYaw = telemetry_data[16];
}

static void simulateTime()
{
    static double start_time;
    struct timeval tv = {};
    gettimeofday(&tv, NULL);
    double curr_time = (double)(tv.tv_sec + tv.tv_usec/1e6);
    if (start_time == 0) {
        start_time = curr_time;
    }
    time = curr_time - start_time;
}

int main (int argc, char *argv[])
{
    udp_client_socket_init(&motor_client_socket, HOST, MOTOR_PORT, 0);

    udp_socket_t telemetry_server_socket = {};
    udp_server_socket_init(&telemetry_server_socket, TELEMETRY_PORT, 0);

    printf("Hit the start button ...\n");
    fflush(stdout);

    while (true) {

        double telemetry_data[17] = {};

        if (udp_receive_data(
                    telemetry_server_socket,
                    telemetry_data,
                    sizeof(telemetry_data))) {

            simulateAltimeter(telemetry_data);
            simulateQuaternion(telemetry_data);
            simulateGyrometer(telemetry_data);
            simulateOpticalFlow(telemetry_data);
            simulateReceiver(telemetry_data);

            simulateTime();

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
