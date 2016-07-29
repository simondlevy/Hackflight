/*
   socketutils.hpp: class declarations for socket utilities

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with 3DSLAM.  If not, see <http://www.gnu.org/licenses/>.
*/

class SocketServer {

    private:

        char hostname[100];
        int port;
        int sockfd;
        int clientfd;

    public:

        SocketServer(const char * hostname = "localhost", int port = 20000);

        void acceptConnection(void);

        int recv(char * buf, int count);

        void send(char * buf, int count);

        void halt(void); 
};

class SocketClient {

    private:

        char hostname[100];
        int port;
        int sockfd;

    public:

        SocketClient(const char * hostname = "localhost", int port = 20000);

        void connectToServer(void);

        int available(void);

        int recv(char * buf, int count);

        void send(char * buf, int count);

        void halt(void); 
};
