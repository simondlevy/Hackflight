/* UDP socket support
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdbool.h>
#include <sys/socket.h>
#include <netdb.h>

typedef struct {

    int sock;
    struct sockaddr_in si_other;

} udp_socket_t;

void udp_close_connection(int sock);

void udp_client_socket_init(udp_socket_t * udp_socket, const char * host, const short port, unsigned int timeoutMsec);

void udp_server_socket_init(udp_socket_t * udp_socket, const short port, unsigned int timeoutMsec);

void udp_set_timeout(udp_socket_t udp_socket, uint32_t msec);

void udp_send_data(udp_socket_t udp_socket, void * buf, size_t len);

bool udp_receive_data(udp_socket_t udp_socket, void * buf, size_t len);
