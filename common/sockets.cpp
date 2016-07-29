/*
   sockets.cpp: Implementation of socket classes

   Copyright (C) Simon D. Levy 2016

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

#include "sockets.hpp"

#include <stdio.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

// http://web.eecs.utk.edu/~plank/plank/classes/cs360/360/notes/Sockets/sockettome.c
static int serve_socket(const char *hostname, int port)
{
    int sockfd;
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

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket()");
        exit(1);
    }

    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));


    if (bind(sockfd, (struct sockaddr *)&sn, sizeof(sn)) == -1) {
        perror("bind()");
        exit(1);
    }

    return sockfd;
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

SocketServer::SocketServer(const char * _hostname, int _port)
{
    strcpy(this->hostname, _hostname);
    this->port = _port;
}


void SocketServer::acceptConnection(void)
{
    this->sockfd = serve_socket(this->hostname, this->port);

    printf("Listening for %s:%d\n", this->hostname, this->port);

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
    printf("Halting %s:%d\n", this->hostname, this->port);

    close(this->sockfd);
    close(this->clientfd);
}

SocketClient::SocketClient(const char * _hostname, int _port)
{
    strcpy(this->hostname, _hostname);
    this->port = _port;
}

void SocketClient::connectToServer(void)
{
    // http://web.eecs.utk.edu/~plank/plank/classes/cs360/360/notes/Sockets/sockettome.c
    struct sockaddr_in sn;
    struct hostent *he;
    if (!(he = gethostbyname(this->hostname))) {
        printf("can't get host id for %s\n", this->hostname);
    }
    int ok = 0;
    this->sockfd = 0;
    while (!ok) {
        sn.sin_family = AF_INET;
        sn.sin_port  = htons(port);
        sn.sin_addr.s_addr = *(u_long*)(he->h_addr_list[0]);

        if ((this->sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket()");
        }
        ok = (connect(this->sockfd, (struct sockaddr*)&sn, sizeof(sn)) != -1);
        if (!ok) sleep (1);
    }  
}

int SocketClient::available(void)
{
    int avail;
    ioctl(this->sockfd, FIONREAD, &avail);
    return avail;
}

int SocketClient::recv(char * buf, int count)
{
    return read_from_socket(this->sockfd, buf, count);
}

void SocketClient::send(char * buf, int count)
{
    write(this->sockfd, buf, count);
}

void SocketClient::halt(void)
{
    close(this->sockfd);
}


