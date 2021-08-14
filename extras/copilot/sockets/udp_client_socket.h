/*
 * Class for UDP client sockets
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "udp_socket.h"

#include <string.h>
#include <time.h>

class UdpClientSocket : public UdpSocket {

    public:

        UdpClientSocket(const char * host, const short port, const uint32_t timeoutMsec=0)
        {
            // Create socket
            _sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (_sock == SOCKET_ERROR) {
                sprintf_s(_message, "socket() failed");
                return;
            }

            // Setup address structure
            memset((char *)&_si_other, 0, sizeof(_si_other));
            _si_other.sin_family = AF_INET;
            _si_other.sin_port = htons(port);
            Socket::inetPton(host, _si_other);

            // Check for / set up optional timeout for receiveData
            UdpSocket::setUdpTimeout(timeoutMsec);
        }
};
