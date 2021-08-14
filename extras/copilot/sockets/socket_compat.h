/*
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdio.h>

typedef int SOCKET;

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

static const int INVALID_SOCKET = -1;
static const int SOCKET_ERROR   = -1;

class Socket {

    protected:

        SOCKET _sock;

        char _message[200];

        void inetPton(const char * host, struct sockaddr_in & saddr_in)
        {
            inet_pton(AF_INET, host, &(saddr_in.sin_addr));
        }

        void setUdpTimeout(uint32_t msec)
        {
            struct timeval timeout;
            timeout.tv_sec = msec / 1000;
            timeout.tv_usec = (msec * 1000) % 1000000;
            setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        }

    public:

        void closeConnection(void)
        {
            close(_sock);
        }

        char * getMessage(void)
        {
            return _message;
        }
};
