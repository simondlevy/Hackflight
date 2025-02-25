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

// Framework
#include <proc_net.hpp>

// Hackflight
#include <posix/serial.hpp>
#include <msp/parser.hpp>
#include <msp/serializer.hpp>

static const uint16_t LOGGING_CLIENT_PORT = 9000;

static const uint16_t SPIKE_CLIENT_PORT = 8100;

static const uint32_t CLIENT_FREQ_HZ = 100;

static ProcNet proc_net;

static bool spike_client_connected;

static void * spike_thread_fun(void * arg)
{
    proc_net.serve_visualizer(SPIKE_CLIENT_PORT);

    while (true) {

        proc_net.accept_client();

        spike_client_connected = true;

        while (spike_client_connected) {

            // Yield to other threads
            usleep(1000); 
        }
    }

    return NULL;
}

/*
static bool logging_client_connected;

static void * logging_thread_fun(void * arg)
{
    proc_net.serve_visualizer(LOGGING_CLIENT_PORT);

    while (true) {

        proc_net.accept_client();

        logging_client_connected = true;

        while (logging_client_connected) {

            // Yield to other threads
            usleep(1000); 
        }
    }

    return NULL;
}*/

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

    // Start a thread for spike telemetry
    pthread_t spike_thread = {};
    pthread_create(&spike_thread, NULL, spike_thread_fun, NULL);

    // Parser accepts messages from Teensy
    hf::MspParser parser = {};

    // Serializer sends messages back to Teensy
    hf::MspSerializer serializer = {};

    long msec_prev = 0;

    while (true) {

        char byte = 0;

        struct timeval time;

        gettimeofday(&time, NULL);

        long msec_curr = 1000 * time.tv_sec + time.tv_usec / 1000;

        if (read(fd, &byte, 1) == 1) {

            if (parser.parse(byte) == 121) {

                // Add spikes to network
                for (uint8_t k=0; k<parser.getByte(0); ++k) {
                    proc_net.add_spike(parser.getByte(2*k+1), parser.getByte(2*k+2));

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
                    if (spike_client_connected && !proc_net.send_counts_to_visualizer()) {
                        spike_client_connected = false;
                    }

                    msec_prev = msec_curr;
                }

                // Reset for next time
                proc_net.clear();
            }
        }
    }

    return 0;
}
