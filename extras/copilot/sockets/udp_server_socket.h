/*
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "udp_socket.h"

static void error(const char * message)
{
    fprintf(stderr, "%s\n", message);
    exit(1);
}

void udp_server_socket_init(udp_socket_t * udp_socket, const short port, unsigned int timeoutMsec)
{
    // Create socket
    udp_socket->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket->sock == INVALID_SOCKET) {
        error("socket failed()");
    }

    // Prepare the sockaddr_in structure
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);

    // Bind
    if (bind(udp_socket->sock, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR) {
        error("bind() failed");
    }

    // Check for / set up optional timeout for receiveData
    setUdpTimeout(udp_socket->sock, timeoutMsec);
}
