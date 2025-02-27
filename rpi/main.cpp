/*
   RaspberryPi code for Hackflight

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
#include <pthread.h>

#include <framework/include/utils/levy/proc_net.hpp>

#include <posix-utils/serial.hpp>
#include <posix-utils/server.hpp>

#include <hackflight/src/msp/parser.hpp>
#include <hackflight/src/msp/serializer.hpp>
#include <hackflight/src/msp/messages.hpp>

static const uint16_t LOGGING_CLIENT_PORT = 9000;

static const uint16_t SPIKE_CLIENT_PORT = 8100;

static const uint32_t CLIENT_FREQ_HZ = 100;

static ProcNet proc_net;
//
// Parser accepts messages from Teensy
static hf::MspParser parser;

// Serializer sends messages back to Teensy
static hf::MspSerializer serializer;


static void handleSpikes(
        const int fd, const long msec_curr, Server & spikeServer)
{
    static long msec_prev;

    // Add spikes to network
    for (uint8_t k=0; k<parser.getByte(0); ++k) {
        proc_net.add_spike(
                parser.getByte(2*k+1), parser.getByte(2*k+2));

    }

    // Run the spikes through the processor/network
    vector<int> counts;
    vector<double> times;
    proc_net.get_counts_and_times(counts, times);

    // Send the network output back to the Teensy for decoding
    uint8_t msg[3] = {1, (uint8_t)counts[0], (uint8_t)times[0]};
    serializer.serializeBytes(121, msg, 3);
    write(fd, serializer.payload, serializer.payloadSize);

    // Periodically send the spike counts to the client
    if (msec_curr - msec_prev > 1000/CLIENT_FREQ_HZ) {

        // Check for spike client disconnect
        uint8_t counts[256] = {};
        const auto ncounts = proc_net.get_counts(counts);
        spikeServer.sendData(counts, ncounts);
        msec_prev = msec_curr;
    }

    // Reset for next time
    proc_net.clear();

}

static void handleState(const long msec_curr, Server & spikeServer)
{
    printf("phi=%+3.3f  theta=%+3.3f  psi=%+3.3f\n",
            parser.getFloat(6), parser.getFloat(8), parser.getFloat(10));
}

int main(int argc, char ** argv)
{
    if (argc < 2) {
        printf("Usage: %s NETWORK\n", argv[0]);
        exit(1);
    }

    proc_net.load(argv[1], "risp");

    proc_net.clear();

    // Open a serial connection to the Teensy
    auto fd = openSerialPort("/dev/ttyS0", B115200);

    if (fd < 0) {
        return 1;
    }

    auto loggingServer = Server(LOGGING_CLIENT_PORT, "logging");

    auto spikeServer = Server(SPIKE_CLIENT_PORT, "spike");

    while (true) {

        char byte = 0;

        struct timeval time;

        gettimeofday(&time, NULL);

        long msec_curr = 1000 * time.tv_sec + time.tv_usec / 1000;

        if (read(fd, &byte, 1) == 1) {

            switch (parser.parse(byte)) {

                case hf::MSP_SPIKES:
                    handleSpikes(fd, msec_curr, spikeServer);
                    break;

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
