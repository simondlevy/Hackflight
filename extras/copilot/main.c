/*
  Copyright(C) 2021 Simon D.Levy

  MIT License
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "udp_sockets.h"
#include "hackflight.h"

static const char * HOST = "127.0.0.1";
static const uint16_t MOTOR_PORT = 5000;
static const uint16_t TELEMETRY_PORT = 5001;

// Avaiable to Copilot
double receiverThrottle = 0;

static udp_socket_t motor_client_socket; 

// Called by Copilot
void runMotor(double value)
{
    double motors[4] = {value, value, value, value};
    udp_send_data(motor_client_socket, motors, sizeof(motors));
}

int main (int argc, char *argv[])
{
    udp_client_socket_init(&motor_client_socket, HOST, MOTOR_PORT, 0);

    udp_socket_t telemetry_server_socket = {};
    udp_server_socket_init(&telemetry_server_socket, TELEMETRY_PORT, 0);

    printf("Hit the start button ...");
    fflush(stdout);

    while (true) {

        double telemetry_data[17] = {};

        if (udp_receive_data(
                    telemetry_server_socket,
                    telemetry_data,
                    sizeof(telemetry_data))) {

            receiverThrottle = telemetry_data[13];

            udp_set_timeout(telemetry_server_socket, 100);

            if (telemetry_data[0] < 0) {
                udp_close_connection(motor_client_socket);
                udp_close_connection(telemetry_server_socket);
                break;
            }

            // Calls Copilot
            step();
        }

        else {

            fprintf(stderr, "failed to receive\n");
            fflush(stderr);
        }
    }

    return 0;
}
