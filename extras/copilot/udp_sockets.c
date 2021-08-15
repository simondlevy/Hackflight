/* UDP socket support
 *
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "udp_sockets.h"

static const int INVALID_SOCKET = -1;
static const int SOCKET_ERROR   = -1;

static const int SLEN = (int)sizeof(struct sockaddr_in);

static void _set_timeout(int sock, uint32_t msec)
{
    struct timeval timeout;
    timeout.tv_sec = msec / 1000;
    timeout.tv_usec = (msec * 1000) % 1000000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

static void _error(const char * message)
{
    fprintf(stderr, "%s\n", message);
    exit(1);
}

void udp_close_connection(udp_socket_t udp_socket)
{
    close(udp_socket.sock);
}

void udp_client_socket_init(
        udp_socket_t * udp_socket,
        const char * host,
        const short port,
        unsigned int timeoutMsec)
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
    inet_pton(AF_INET, host, &(udp_socket->si_other.sin_addr));

    // Check for / set up optional timeout for receiveData
    _set_timeout(udp_socket->sock, timeoutMsec);
}

void udp_server_socket_init(
        udp_socket_t * udp_socket,
        const short port,
        unsigned int timeoutMsec)
{
    // Create socket
    udp_socket->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket->sock == INVALID_SOCKET) {
        _error("socket failed()");
    }

    // Prepare the sockaddr_in structure
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);

    // Bind
    if (bind(udp_socket->sock,
             (struct sockaddr *)&server,
             sizeof(server)) == SOCKET_ERROR) {
        _error("bind() failed");
    }

    // Check for / set up optional timeout for receiveData
    _set_timeout(udp_socket->sock, timeoutMsec);
}

void udp_set_timeout(udp_socket_t udp_socket, uint32_t msec)
{
    if (msec > 0) {
        _set_timeout(udp_socket.sock, msec);
    }
}

void udp_send_data(udp_socket_t udp_socket, void * buf, size_t len)
{
    sendto(udp_socket.sock,
           (const char *)buf,
           (int)len,
           0,
           (struct sockaddr *)&(udp_socket.si_other),
           SLEN);

}

bool udp_receive_data(udp_socket_t udp_socket, void * buf, size_t len)
{
    unsigned int recvd = 0;

    return recvfrom(udp_socket.sock,
                    (char *)buf,
                    (int)len,
                    0,
                    (struct sockaddr *) &(udp_socket.si_other),
                    &recvd)
        == len;
}
