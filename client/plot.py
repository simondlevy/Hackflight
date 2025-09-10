#!/usr/bin/python3

'''
Copyright (C) 2025 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import argparse
from argparse import ArgumentDefaultsHelpFormatter
import socket
import sys
from threading import Thread
from time import sleep
import matplotlib.pyplot as plt
import matplotlib

from realtime_plot import RealtimePlotter

try:
    from __msp__ import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)


BLUETOOTH_ADDRESSES = {
    'bolt': '64:B7:08:93:71:1E',
    'teensy': '64:B7:08:86:F2:AE'
}

BLUETOOTH_PORT = 1

ANGLE_MAX = 60
PSI_MAX = 180
Z_MAX = 1


class TelemetryPlotter(RealtimePlotter):

    def __init__(self, name):

        z_range = 0, Z_MAX
        z_ticks = 0, 0.5, Z_MAX

        angle_range = -ANGLE_MAX, ANGLE_MAX
        angle_ticks = -ANGLE_MAX, 0, ANGLE_MAX

        psi_range = -PSI_MAX, PSI_MAX
        psi_ticks = -PSI_MAX, 0, PSI_MAX

        RealtimePlotter.__init__(
                self,
                [z_range, angle_range, angle_range, psi_range], 
                window_name=name,
                yticks = [z_ticks, angle_ticks, angle_ticks, psi_ticks],
                ylabels = ['$z$', '$\phi$', '$\\theta$', '$\psi$'])

        self.z = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0

    def getValues(self):

         return self.z, self.phi, self.theta, self.psi


class LoggingParser(MspParser):

    def __init__(self, client, plotter):

        MspParser.__init__(self)
        self.client = client
        self.plotter = plotter
        self.running = True
        self.spike_viz_client = None

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        self.plotter.z = z
        self.plotter.phi = phi
        self.plotter.theta = theta
        self.plotter.psi = psi

    def handle_SPIKES(self, n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11,
                      n12, n13, n14, n15):

        if self.spike_viz_client is not None:

            msg = (('{"Event Counts":[%d,%d,%d,%d,%d,%d,%d], ' +
                    '"Neuron Alias":[0,1,2,3,4,5,6]}\n') %
                   (n0, n1, n2, n3, n4, n5, n6))

            try:
                self.spike_viz_client.send(msg.encode())

            except Exception as e:
                pass


def logging_threadfun(parser):

    while parser.running:

        try:

            parser.parse(parser.client.recv(1))

        except Exception as e:
            print('Failed to receiving logging data: ' + str(e))
            parser.running = False
            return


def connect_to_server(name, port):

    addr = BLUETOOTH_ADDRESSES[name]

    while True:

        try:

            print('Connecting to server %s:%d ... ' % (addr, port), end='')
            sys.stdout.flush()

            # Create a Bluetooth or IP socket depending on address format
            client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                    socket.BTPROTO_RFCOMM)
                      if ':' in addr
                      else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

            try:
                client.connect((addr, port))
                print(' connected')
                break

            except Exception as e:
                print(str(e) + ': is server running?')
                sleep(1)

        except KeyboardInterrupt:
            break

    return client


def main():

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-b', '--bluetooth-server',
                           choices=['bolt', 'teensy'],
                           default='bolt', help='Bluetooth server')

    args = argparser.parse_args()

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 18}

    matplotlib.rc('font', **font)

    matplotlib.pyplot.rcParams['text.usetex'] = True  # Enable LaTeX

    client = connect_to_server(args.bluetooth_server, BLUETOOTH_PORT)

    plotter = TelemetryPlotter(args.bluetooth_server)

    parser = LoggingParser(client, plotter)
    thread = Thread(target=logging_threadfun, args=(parser, ))
    thread.daemon = True
    thread.start()

    plotter.start()

main()
