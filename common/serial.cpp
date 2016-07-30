/*
   serial.cpp : Implementation of SerialConnection class

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
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include "serial.hpp"

// Adapted from http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

static int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

static void set_blocking (int fd, bool should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        fprintf(stderr, "error %d setting term attributes", errno);
}

SerialConnection::SerialConnection(const char * portname, int baudrate, bool blocking, int parity)
{
    strcpy(this->portname, portname);
    this->blocking = blocking;
    this->parity = parity;

    switch (baudrate) {
        case 110:
            this->baudrate = B110;
            break;
        case 300:
            this->baudrate = B300;
            break;
        case 600:
            this->baudrate = B600;
            break;
        case 1200:
            this->baudrate = B1200;
            break;
        case 2400:
            this->baudrate = B2400;
            break;
        case 4800:
            this->baudrate = B4800;
            break;
        case 9600:
            this->baudrate = B9600;
            break;
        case 19200:
            this->baudrate = B19200;
            break;
        case 38400:
            this->baudrate = B38400;
            break;
        case 57600:
            this->baudrate = B57600;
            break;
        case 115200:
            this->baudrate = B115200;
            break;
        default:
            fprintf(stderr, "unrecognized baudrate %d\n", baudrate);
            return;
    }

}

void SerialConnection::openConnection(void)
{

    this->fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (this->fd < 0) {
        fprintf(stderr, "error %d opening %s: %s\n", errno, this->portname, strerror (errno));
        exit(1);
    }

    set_interface_attribs (this->fd, this->baudrate, this->parity);

    set_blocking (this->fd, this->blocking);
}

int SerialConnection::bytesAvailable(void)
{
    int avail = 0;
    ioctl(this->fd, FIONREAD, &avail);
    return avail;
}

int SerialConnection::readBytes(char * buf, int size)
{
    return read(this->fd, buf, size);
}

int SerialConnection::writeBytes(char * buf, int size)
{
    return write(this->fd, buf, size);
}

void SerialConnection::closeConnection(void)
{
    close(this->fd);
}
