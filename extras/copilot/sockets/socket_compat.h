/*
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

static const int INVALID_SOCKET = -1;
static const int SOCKET_ERROR   = -1;

typedef struct {

    int sock;

    char message[200];

} socket_t;


void inetPton(const char * host, struct sockaddr_in saddr_in)
{
    inet_pton(AF_INET, host, &(saddr_in.sin_addr));
}

void setUdpTimeout(socket_t socket, uint32_t msec)
{
    struct timeval timeout;
    timeout.tv_sec = msec / 1000;
    timeout.tv_usec = (msec * 1000) % 1000000;
    setsockopt(socket.sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

void closeConnection(socket_t socket)
{
    close(socket.sock);
}

char * getMessage(socket_t socket)
{
    return socket.message;
}
