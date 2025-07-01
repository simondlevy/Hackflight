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

#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>

#include <posix-utils/server.hpp>
#include <posix-utils/serial.hpp>

#include <msp/parser.hpp>
#include <msp/messages.h>
#include <msp/serializer.hpp>

#include "tick.hpp"

// NB, Bluetooth
static const uint16_t RADIO_PORT = 1;

// Serial connection to FC
static int serialfd;

// Current altitude (m)
static float state_z;

static void * control_fun(void * arg)
{
    // Parser accepts messages from flight controller (FC)
    MspParser parser = {};

    // Loop forever, reading state messages from FC
    while (true) {

        char byte = 0;

        if (read(serialfd, &byte, 1) == 1 && 
                parser.parse(byte) == MSP_STATE_Z) {
            state_z = parser.getFloat(0);
        }
    }

    return NULL;
}

static void sendPayload(
        const int serialfd, const uint8_t * payload, const uint8_t size)
{
    const auto ignore = write(serialfd, payload, size);
    (void)ignore;
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

            // Special handling for hover setpoint messages
            if (msgid == MSP_SET_SETPOINT_HOVER) {

                tick();

                const float zerror = parser.getFloat(3) - state_z;

                // Grab the setpoint values and replace the altitude setpoint
                // with its error
                const float setpoint[4] = {
                    parser.getFloat(0),
                    parser.getFloat(1),
                    parser.getFloat(2),
                    zerror
                };

                // Send the modified setpoint to the flight controller
                MspSerializer serializer = {};
                serializer.serializeFloats(MSP_SET_SETPOINT_HOVER, setpoint, 4);
                sendPayload(serialfd, serializer.payload, serializer.payloadSize);
            }

            // Otherwise, send the unmodified payload along to the flight
            // controller
            else {
                uint8_t payload[256] = {};
                const auto payloadSize = parser.getPayload(payload);
                sendPayload(serialfd, payload, payloadSize);
            }
        }

        sleep(0); // yield
    }

    return 0;
}
