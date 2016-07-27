/*
   socketutils.cpp: socket utilities for HackflightSim

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

#include "socketutils.hpp"

#include <stdio.h>
#include <netdb.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

// http://web.eecs.utk.edu/~plank/plank/classes/cs360/360/notes/Sockets/sockettome.c
static int serve_socket(const char *hostname, int port)
{
    int s;
    struct sockaddr_in sn;
    struct hostent *he;

    if (!(he = gethostbyname(hostname))) {
        puts("can't gethostname");
        exit(1);
    }

    memset((char*)&sn, 0, sizeof(sn));
    sn.sin_family = AF_INET;
    sn.sin_port = htons((short)port);
    sn.sin_addr.s_addr = htonl(INADDR_ANY);

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket()");
        exit(1);
    }

    if (bind(s, (struct sockaddr *)&sn, sizeof(sn)) == -1) {
        perror("bind()");
        exit(1);
    }

    return s;
}

static int accept_connection(int sockfd)
{
    int x;
    struct sockaddr_in sn;

    if(listen(sockfd, 1) == -1) {
        perror("listen()");
        exit(1);
    }

    bzero((char *)&sn, sizeof(sn));
    
    if((x = accept(sockfd, (struct sockaddr *)NULL, NULL)) == -1) {
        perror("accept()");
        exit(1);
    }

    return x;
}

static int read_from_socket(int clientfd, char * buf, int n)
// http://developerweb.net/viewtopic.php?id=2933
{
    fd_set readset;
    int result = 0;

    do {
        FD_ZERO(&readset);
        FD_SET(clientfd, &readset);
        result = select(clientfd + 1, &readset, NULL, NULL, NULL);
    } while (result == -1 && errno == EINTR);

    if (result > 0) {

        if (FD_ISSET(clientfd, &readset)) {

            // The clientfd has data available to be read 
            result = recv(clientfd, buf, n, 0);

            if (result == 0) {
                // This means the other side closed the socket
                close(clientfd);
            }

            else {
                return n; // Success!
            }
        }
    }

    return 0;
}

static int connect_to_server(int port, const char * hostname)
{
    // http://web.eecs.utk.edu/~plank/plank/classes/cs360/360/notes/Sockets/sockettome.c
    struct sockaddr_in sn;
    struct hostent *he;
    if (!(he = gethostbyname(hostname))) {
        printf("can't get host id for %s\n", hostname);
    }
    int ok = 0;
    int sockfd = 0;
    while (!ok) {
        sn.sin_family = AF_INET;
        sn.sin_port  = htons(port);
        sn.sin_addr.s_addr = *(u_long*)(he->h_addr_list[0]);

        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket()");
        }
        ok = (connect(sockfd, (struct sockaddr*)&sn, sizeof(sn)) != -1);
        if (!ok) sleep (1);
    }  
    return sockfd;
}

SocketServer::SocketServer(void) 
{
}


void SocketServer::connect(const char * hostname, int port)
{
    this->sockfd = serve_socket(hostname, port);

    printf("Listening for client on host %s at port %d\n", hostname, port);

    this->clientfd = accept_connection(sockfd);

    printf("Accepted connection\n");
}

int SocketServer::recv(char * buf, int count)
{
    return read_from_socket(this->clientfd, buf, count);
}

void SocketServer::send(char * buf, int count)
{
    write(this->clientfd, buf, count);
}

void SocketServer::halt(void)
{
    close(this->sockfd);
    close(this->clientfd);
}
