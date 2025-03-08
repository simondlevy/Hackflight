/*
   RaspberryPi code for Hackflight

   Additional installs required:

     Clone https://github.com/simondlevy/posix-utils to ~/Desktop

     sudo apt install libbluetooth-dev

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
#include <sys/ioctl.h>

#include <posix-utils/serial.hpp>
#include <posix-utils/server.hpp>

#include <hackflight/src/msp/parser.hpp>
#include <hackflight/src/msp/serializer.hpp>
#include <hackflight/src/msp/messages.hpp>

// NB, Bluetooth
static const uint16_t LOGGING_CLIENT_PORT = 1;

static const uint32_t CLIENT_FREQ_HZ = 100;

//
// Parser accepts messages from Teensy
static hf::MspParser parser;

// Serializer sends messages back to Teensy
//static hf::MspSerializer serializer;

static void handleState(const long msec_curr, Server & stateServer)
{
    const float state[12] = {
        parser.getFloat(0),
        parser.getFloat(1),
        parser.getFloat(2),
        parser.getFloat(3),
        parser.getFloat(4),
        parser.getFloat(5),
        parser.getFloat(6),
        parser.getFloat(7),
        parser.getFloat(8),
        parser.getFloat(9),
        parser.getFloat(10),
        parser.getFloat(11)
    };

    stateServer.sendData((uint8_t *)state, 12 * sizeof(float));
}

int main(int argc, char ** argv)
{
    // Open a serial connection to the Teensy
    auto fd = Serial::open_port("/dev/ttyS0", B115200);

    if (fd < 0) {
        return 1;
    }

    // true = Bluetooth
    auto loggingServer = Server(LOGGING_CLIENT_PORT, true);

    while (true) {

        char byte = 0;

        struct timeval time;

        gettimeofday(&time, NULL);

        long msec_curr = 1000 * time.tv_sec + time.tv_usec / 1000;

        if (read(fd, &byte, 1) == 1) {

            switch (parser.parse(byte)) {

                case hf::MSP_STATE:
                    handleState(msec_curr, loggingServer);
                    break;

                default:
                    break;

            }
        }
    }

    return 0;
}
