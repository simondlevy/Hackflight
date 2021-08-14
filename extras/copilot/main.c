/*
  Copyright(C) 2021 Simon D.Levy

  MIT License
*/

#include <stdio.h>
#include <stdint.h>

#include "udp_sockets.h"

static const char * HOST = "127.0.0.1";
static const uint16_t MOTOR_PORT = 5000;
static const uint16_t TELEMETRY_PORT = 5001;

int main (int argc, char *argv[])
{
    udp_socket_t motor_client_socket = {};
    udp_client_socket_init(&motor_client_socket, HOST, MOTOR_PORT, 0);

    udp_socket_t telemetry_server_socket = {};
    udp_server_socket_init(&telemetry_server_socket, TELEMETRY_PORT, 0);

    printf("Hit the start button ...");

    while (true) {

        double telemetry_bytes[17] = {};

        receiveData(telemetry_server_socket, telemetry_bytes, sizeof(telemetry_bytes));

        setTimeout(telemetry_server_socket, 100);

        if (telemetry_bytes[0] < 0) {
            break;
        }
    }

    return 0;
}
