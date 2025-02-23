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
#include <time.h>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>

using namespace std;

#include <proc_net.hpp>

static const uint16_t VIZ_PORT = 8100;

static const uint32_t CLIENT_FREQ_HZ = 100;

// Adapted from https://www.geeksforgeeks.org/serial-port-connection-in-cpp,
//   https://stackoverflow.com/questions/8070632
static int openSerialPort(const char* portname, int speed)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        cerr << "Error opening " << portname << ": "
             << strerror(errno) << endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Error from tcgetattr: " << strerror(errno) << endl;
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
        cerr << "Error from tcsetattr: " << strerror(errno) << endl;
        return -1;
    }

    return fd;
}

static ProcNet proc_net;

static int fd;

static bool connected;

static void * threadfun(void * arg)
{
    // Support a little state machine for parsing incoming spikes
    uint8_t state = 0;

    long msec_prev = 0;

    while (true) {

        char byte = 0;

        struct timeval time;

        gettimeofday(&time, NULL);

        long msec_curr = 1000 * time.tv_sec + time.tv_usec / 1000;

        if (read(fd, &byte, 1) == 1) {

            static uint8_t neuron_id;

            switch (state) {

                // Startup
                case 0:

                    if (byte == 0xFF) {
                        state = 1;
                    }
                    break;

                case 1:

                    // Got an entire spike-train message
                    if (byte == 0xFF) {

                        // Run the spikes through the processor/network
                        vector<int> counts;
                        vector<double> times;
                        proc_net.get_counts_and_times(counts, times);

                        // Send the network output back to the Teensy for decoding
                        uint8_t out[2] = {(uint8_t)counts[0], (uint8_t)times[0]};
                        auto tmp = write(fd, out, 2);
                        (void)tmp;

                        // Periodically send the spike counts to the client
                        if (msec_curr - msec_prev > 1000/CLIENT_FREQ_HZ) {

                            // Check for client disconnect
                            if (connected && !proc_net.send_counts_to_visualizer()) {
                                connected = false;
                            }

                            msec_prev = msec_curr;
                        }

                        // Reset for next time
                        proc_net.clear();
                    }

                    // Still just parsing the neuron ID part
                    else {
                        neuron_id = byte;
                        state = 2;
                    }
                    break;

                case 2:

                    // Got an ID and a time; add a spike and keep parsing
                    proc_net.add_spike(neuron_id, byte);
                    state = 1;
                    break;
            }
        }
    }

    return NULL;
}


int main(int argc, char ** argv)
{
    if (argc < 2) {
        printf("Usage: %s NETWORK\n", argv[0]);
        exit(1);
    }

    proc_net.load(argv[1], "risp");

    proc_net.clear();

    proc_net.serve_visualizer(VIZ_PORT);

    // Open a serial connection to the Teensy
    fd = openSerialPort("/dev/ttyS0", B115200);

    if (fd < 0) {
        return 1;
    }

    // Start a thread for serial connection with Teensy
    pthread_t thread = {};
    pthread_create(&thread, NULL, threadfun, &proc_net);

    while (true) {

        proc_net.accept_client();

        connected = true;

        while (connected) {

            // Yield to other thread
            usleep(1000); 
        }
    }


    return 0;
}
