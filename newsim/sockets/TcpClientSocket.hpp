/*
  Class for TCP client sockets
 
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


class TcpClientSocket : public TcpSocket {

    public:

        TcpClientSocket(const char * host, const short port)
            : TcpSocket(host, port)        
        {
        }

        void openConnection(void)
        {
            // Connect to server, returning on failure
            if (connect(
                        _sock,
                        _addressInfo->ai_addr,
                        (int)_addressInfo->ai_addrlen) == SOCKET_ERROR) {

                closesocket(_sock);
                _sock = INVALID_SOCKET;
                sprintf_s(
                        _message, 
                        "connect() failed; please make sure server is running");
                return;
            }

            // For a client, the connection is the same as the main socket
            _conn = _sock;

            // Success!
            _connected = true;
        }
};
