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

#include <posix-utils/server.hpp>
#include <posix-utils/serial.hpp>

#include <datatypes.h>
#include <msp/parser.hpp>
#include <msp/messages.h>
#include <tennlab/differencer.hpp>

// NB, Bluetooth
static const uint16_t RADIO_PORT = 1;
//static const uint16_t SPIKE_PORT = 3;

// Serial connection to FC
static int serialfd;

static demands_t hover_demands;

static void * control_fun(void * arg)
{
    // Parser accepts messages from flight controller (FC)
    MspParser parser = {};

    // Loop forever, reading state messages from FC
    while (true) {

        char byte = 0;

        if (read(serialfd, &byte, 1) == 1 && 
                parser.parse(byte) == MSP_STATE_Z) {
            const float z = parser.getFloat(0);
            printf("z=%3.3f\n", z);
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

    pthread_t control_thread = {};
    pthread_create(&control_thread, NULL, control_fun, NULL);

    // true = Bluetooth
    auto setpointServer = Server(RADIO_PORT, "setpoint", true);

    MspParser parser = {};

    // Loop forever, retreiving setpoint messages from the client and sending
    // them to the flight controller
    while (true) {

        uint8_t byte = 0;

        setpointServer.receiveData(&byte, 1);

        const uint8_t msgid = parser.parse(byte);

        // If we've got a new message
        if (msgid) {

            // Get the message payload and send it to the flight controller
            uint8_t payload[256] = {};
            const auto payloadSize = parser.getPayload(payload);
            const auto ignore = write(serialfd, payload, payloadSize);
            (void)ignore;

            // If the message is a hover setpoint, store the demands
            if (msgid == MSP_SET_SETPOINT_HOVER) {
                hover_demands.thrust = parser.getFloat(0);
                hover_demands.roll = parser.getFloat(1);
                hover_demands.pitch = parser.getFloat(2);
                hover_demands.yaw = parser.getFloat(3);
            }
        }

        sleep(0); // yield
    }

    return 0;
}
