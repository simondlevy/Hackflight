/*
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdbool.h>

#include "posix_sockets.h"

static const int SLEN = (int)sizeof(struct sockaddr_in);

typedef struct {

    int sock;
    struct sockaddr_in si_other;

} udp_socket_t;

void setTimeout(udp_socket_t udp_socket, uint32_t msec)
{
    if (msec > 0) {
        setUdpTimeout(udp_socket.sock, msec);
    }
}

void sendData(udp_socket_t udp_socket, void * buf, size_t len)
{
    sendto(udp_socket.sock,
           (const char *)buf,
           (int)len,
           0,
           (struct sockaddr *)&(udp_socket.si_other),
           SLEN);

}

bool receiveData(udp_socket_t udp_socket, void * buf, size_t len)
{
    unsigned int recvd = 0;

    return recvfrom(udp_socket.sock,
                    (char *)buf,
                    (int)len,
                    0,
                    (struct sockaddr *) &(udp_socket.si_other),
                    &recvd)
        == SLEN;
}
