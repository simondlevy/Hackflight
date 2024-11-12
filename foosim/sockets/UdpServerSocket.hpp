/*
  Class for UDP server sockets
 
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

class UdpServerSocket : public UdpSocket {

    public:

        UdpServerSocket(const short port, const uint32_t timeoutMsec=0)
        {
            // Initialize Winsock, returning on failure
            if (!initWinsock()) return;

            // Create socket
            _sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (_sock == INVALID_SOCKET) {
                sprintf_s(_message, "socket() failed");
                return;
            }

            // Prepare the sockaddr_in structure
            struct sockaddr_in server;
            server.sin_family = AF_INET;
            server.sin_addr.s_addr = INADDR_ANY;
            server.sin_port = htons(port);

            // Bind
            if (bind(
                        _sock,
                        (struct sockaddr *)&server,
                        sizeof(server)) == SOCKET_ERROR) {

                sprintf_s(_message, "bind() failed");
                return;
            }

            // Check for / set up optional timeout for receiveData
            UdpSocket::setUdpTimeout(timeoutMsec);
        }

        static UdpServerSocket * free(UdpServerSocket * socket)
        {
            return (UdpServerSocket *)UdpSocket::free(socket);
        }
 };
