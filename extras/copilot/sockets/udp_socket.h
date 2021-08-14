/*
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "posix_sockets.h"

typedef struct {

    socket_t socket;
    struct sockaddr_in si_other;
    socklen_t slen = sizeof(_si_other);

} udp_socket_t;

void setupTimeout(udp_socket_t udp_socket, uint32_t msec)
{
    if (msec > 0) {
        setUdpTimeout(udp_socket.socket, msec);
    }
}

void sendData(udp_socket_t udp_socket, void * buf, size_t len)
{
    sendto(udp_socket.socket.sock,
           (const char *)buf,
           (int)len,
           0,
           (struct sockaddr *)&(udp_socket.si_other),
           (int)udp_socket.slen);

}

bool receiveData(udp_socket_t udp_socket, void * buf, size_t len)
{
    return recvfrom(udp_socket.socket.sock,
                    (char *)buf,
                    (int)len,
                    0,
                    (struct sockaddr *) &(udp_socket.si_other),
                    &(udp_socket.slen))
        == _slen;
}
