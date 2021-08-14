/*
 * Class for UDP client sockets
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdlib.h>
#include <string.h>

#include "udp_socket.h"

void udp_client_socket_init(udp_socket_t * udp_socket, const char * host, const short port, unsigned int timeoutMsec)
{
    // Create socket
    udp_socket->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket->sock == SOCKET_ERROR) {
        fprintf(stderr, "socket() failed");
        exit(1);
    }

    // Setup address structure
    memset((char *)&udp_socket->si_other, 0, SLEN);
    udp_socket->si_other.sin_family = AF_INET;
    udp_socket->si_other.sin_port = htons(port);
    inetPton(host, udp_socket->si_other);

    // Check for / set up optional timeout for receiveData
    setUdpTimeout(udp_socket->sock, timeoutMsec);
}
