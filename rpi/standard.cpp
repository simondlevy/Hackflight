/*
   RaspberryPi Bluetooth server code for Hackflight

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
#include <pthread.h>
#include <time.h>

#include <posix-utils/serial.hpp>
#include <posix-utils/server.hpp>

#include <msp/parser.hpp>
#include <msp/messages.h>

// NB, Bluetooth
static const uint16_t RADIO_PORT = 1;
static const uint16_t STATE_PORT = 2;
static const uint16_t SPIKE_PORT = 3;

// Serial connection to FC
static int serialfd;

static void * logging_fun(void * arg)
{
    // true = Bluetooth
    auto stateServer = Server(STATE_PORT, "state", true);

    // Parser accepts messages from flight controller (FC)
    MspParser parser = {};

    // Loop forever, reading state messages from FC
    while (true) {

        char byte = 0;

        if (read(serialfd, &byte, 1) == 1) {

            switch (parser.parse(byte)) {

                case MSP_STATE:

                    {
                        const float state[10] = { 
                            parser.getFloat(0),
                            parser.getFloat(1),
                            parser.getFloat(2),
                            parser.getFloat(3),
                            parser.getFloat(4),
                            parser.getFloat(5),
                            parser.getFloat(6),
                            parser.getFloat(7),
                            parser.getFloat(8),
                            parser.getFloat(9)
                        };

                        if (stateServer.isConnected()) {

                            stateServer.sendData((uint8_t *)state, 10 * sizeof(float));
                        }

                    }

                    break;
            }
        }
    }

    return NULL;
}

int main(int argc, char ** argv)
{
    // Open a serial connection to the microcontroller
    serialfd = Serial::open_port("/dev/ttyS0", B115200);

    if (serialfd < 0) {
        return 1;
    }

    pthread_t logging_thread = {};
    pthread_create(&logging_thread, NULL, logging_fun, NULL);

    // true = Bluetooth
    auto setpointServer = Server(RADIO_PORT, "setpoint", true);

    MspParser parser = {};

    // Loop forever, retreiving setpoint messages from the client and sending
    // them to the flight controller
    while (true) {

        uint8_t byte = 0;

        setpointServer.receiveData(&byte, 1);

        if (parser.parse(byte)) {

            uint8_t payload[256] = {};

            const auto payloadSize = parser.getPayload(payload);

            const auto ignore = write(serialfd, payload, payloadSize);
            (void)ignore;
        }


        sleep(0); // yield
    }

    return 0;
}
