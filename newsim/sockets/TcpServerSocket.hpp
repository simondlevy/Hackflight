/*
  General-purpose socket server class
 
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

#include "TcpSocket.hpp"

#ifndef _WIN32
static void closesocket(int socket) { close(socket); }
#endif

class TcpServerSocket : public TcpSocket {

    public:

        TcpServerSocket(
                const char * host,
                const uint16_t port,
                const bool nonblock=false)
            : TcpSocket(host, port)        
        {
            // Bind socket to address
            if (bind(_sock,
                        _addressInfo->ai_addr,
                        (int)_addressInfo->ai_addrlen) == SOCKET_ERROR) {

                closesocket(_sock);
                _sock = INVALID_SOCKET;
                sprintf_s(_message, "bind() failed");
            }

            else {

                // Listen for a connection, exiting on failure
                if (listen(_sock, 1)  == -1) {
                    sprintf_s(_message, "listen() failed");
                }

                if (nonblock) {

                    if (!setNonblocking()) {
                        sprintf_s(_message, "setNonblocking() failed");
                    }
                    
                }
            }
        }

        bool acceptConnection(void)
        {
            // Accept connection, setting message on failure
            _conn = accept(_sock, (struct sockaddr *)NULL, NULL);
            if (_conn == SOCKET_ERROR) {
                sprintf_s(_message, "accept() failed");
                return false;
            }

            return true;
        }
};
