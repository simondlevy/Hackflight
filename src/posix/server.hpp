/*
   Threaded socket server class

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <pthread.h>

#include <posix/sockets.hpp>

class Server {

    public:

        void init(const uint16_t port)
        {
            socket.open(port);

            pthread_create(&thread, NULL, thread_fun, this);
        }

        void sendData(const uint8_t * data, const size_t size)
        {
            connected = connected && socket.sendData(data, size);
        }

    private:

        static void * thread_fun(void * arg)
        {
            auto server = (Server *)arg;

            while (true) {

                server->socket.acceptClient();

                server->connected = true;

                while (server->connected) {
                    usleep(1000); // yield
                }
            }

            return NULL;
        }

        pthread_t thread;

        ServerSocket socket;

        bool connected;
};
