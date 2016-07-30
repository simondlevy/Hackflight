/*
   readserial.cpp : Test code for SerialConnection class

   Copyright (C) Simon D. Levy

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

#include <stdio.h>
#include <stdlib.h>

#include "serial.hpp"

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage:   %s PORTNAME BAUDRATE\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyUSB0 57600\n", argv[0]);
        exit(1);
    }

    SerialConnection s(argv[1], atoi(argv[2]));

    s.openConnection();

    while (true) {

        while (s.bytesAvailable()) {
            char c;
            s.readBytes(&c, 1);
            printf("%c\n", c);
        }
    }

    s.closeConnection();

    return 0;
}
