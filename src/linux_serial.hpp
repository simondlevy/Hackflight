/*
   Linux serial support

   Adapted from https://www.geeksforgeeks.org/serial-port-connection-in-cpp,
     https://stackoverflow.com/questions/8070632

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
#include <termios.h>

static int openSerialPort(const char* portname, int speed)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %d\n",  portname, errno);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %d\n", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
            | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= CS8;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %d\n", errno);
        return -1;
    }

    return fd;
}
