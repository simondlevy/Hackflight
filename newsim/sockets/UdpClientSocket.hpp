/*
  Class for UDP client sockets
 
  Copyright (C) 2019 Simon D. Levy
 
  This file is part of SimFlightControl.

  SimFlightControl is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option)
  any later version.

  SimFlightControl is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  SimFlightControl. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "UdpSocket.hpp"

#include <string.h>
#include <time.h>

class UdpClientSocket : public UdpSocket {

    public:

        UdpClientSocket(const char * host, const short port,
                const uint32_t timeoutMsec=0)
        {
            // Initialize Winsock, returning on failure
            if (!initWinsock()) return;

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

        static UdpClientSocket * free(UdpClientSocket * socket)
        {
            return (UdpClientSocket *)UdpSocket::free(socket);
        }
};
