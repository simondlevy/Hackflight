/*
  Class for UDP sockets
 
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

class UdpSocket : public Socket {

    protected:

        struct sockaddr_in _si_other;
        socklen_t _slen = sizeof(_si_other);

        void setupTimeout(uint32_t msec)
        {
            if (msec > 0) {
                Socket::setUdpTimeout(msec);
            }
        }

    public:

        void sendData(void * buf, size_t len)
        {
            sendto(_sock, (const char *)buf, (int)len, 0,
                    (struct sockaddr *) &_si_other, (int)_slen);

        }

        bool receiveData(void * buf, size_t len)
        {
            return recvfrom(_sock, (char *)buf, (int)len, 0,
                    (struct sockaddr *) &_si_other, &_slen) 
                == (recv_size_t)len;
        }

        static UdpSocket * free(UdpSocket * socket)
        {
            socket->closeConnection();
            delete socket;
            return NULL;
        }
};
