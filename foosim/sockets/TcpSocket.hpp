/*
  Cross-platform compatibility superclass for sockets
 
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

#ifdef _WIN32
#include "WindowsSocket.hpp"
#else
#include "LinuxSocket.hpp"
#endif

class TcpSocket : public Socket {

    protected:

        char _host[200];
        char _port[10];

        socket_t _conn;

        struct addrinfo * _addressInfo;

        bool _connected;

        TcpSocket(const char * host, const short port)
        {
            sprintf_s(_host, "%s", host);
            sprintf_s(_port, "%d", port);

            // No connection yet
            _sock = INVALID_SOCKET;
            _connected = false;
            *_message = 0;

            // Initialize Winsock, returning on failure
            if (!initWinsock()) return;

            // Set up client address info
            struct addrinfo hints = {0};
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_STREAM;

            // Resolve the server address and port, returning on failure
            _addressInfo = NULL;
            int iResult = getaddrinfo(_host, _port, &hints, &_addressInfo);
            if ( iResult != 0 ) {
                sprintf_s(
                        _message, "getaddrinfo() failed with error: %d", 
                        iResult);
                cleanup();
                return;
            }

            // Create a socket for connecting to server, returning on failure
            _sock = socket(
                    _addressInfo->ai_family,
                    _addressInfo->ai_socktype,
                    _addressInfo->ai_protocol);
            if (_sock == INVALID_SOCKET) {
                sprintf_s(_message, "socket() failed");
                cleanup();
                return;
            }
        }

    public:

        bool sendData(void *buf, size_t len)
        {
            return (size_t)send(_conn, (const char *)buf, len, 0) == len;
        }

        bool receiveData(void *buf, size_t len)
        {
            return (size_t)recv(_conn, (char *)buf, len, 0) == len;
        }

        bool isConnected()
        {
            return _connected;
        }
};
