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

from gamepad import Gamepad

try:
    from msp import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)


BLUETOOTH_ADDRESSES = {
        'onboard': '64:B7:08:94:2A:32',
        'bench': '64:B7:08:94:28:76'
}

BLUETOOTH_PORT = 1

UPDATE_RATE_HZ = 100


class LoggingParser(MspParser):

    def __init__(self, client, show_state, visualize_spikes):

        MspParser.__init__(self)
        self.client = client
        self.running = True
        self.show_state = show_state
        self.visualize_spikes = visualize_spikes

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        if self.show_state:
            print(('dx=%+03.2f dy=%+03.2f z=%+03.2f dz=%+03.2f ' +
                   'phi=%+5.1f dphi=%+6.1f theta=%+5.1f dtheta=%+6.1f ' +
                   'psi=%+5.1f dpsi=%+5.1f') %
                  (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))

    def handle_SPIKES(self, n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11,
                      n12, n13, n14, n15):

        if self.visualize_spikes:
            print('i1=%d i2=%d s=%d d1=%d d2=%d s2=%d o=%d' %
                  (n0, n1, n2, n3, n4, n5, n6))


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
                           choices=['onboard', 'bench'],
                           default='onboard', help='Bluetooth server')

    argparser.add_argument('-l', '--log-state', action='store_true',
                           help='log vehicle state')

    argparser.add_argument('-s', '--visualize-spikes', action='store_true',
                           help='display spikes')

    args = argparser.parse_args()

    was_armed = False

    client = connect_to_server(args.bluetooth_server, BLUETOOTH_PORT)

    logging = [True]

    parser = LoggingParser(client, args.log_state, args.visualize_spikes)
    thread = Thread(target=logging_threadfun, args=(parser, ))
    thread.daemon = True
    thread.start()

    gamepad = Gamepad()

    while logging[0] and gamepad.running:

        try:

            if gamepad.armed != was_armed:
                client.send(MspParser.serialize_SET_ARMING(gamepad.armed))
                was_armed = gamepad.armed

            if gamepad.hovering:

                client.send(MspParser.serialize_SET_SETPOINT_HOVER(
                    gamepad.vx, gamepad.vy, gamepad.yawrate, gamepad.zdist))

            else:

                client.send(
                        MspParser.serialize_SET_SETPOINT_RPYT(gamepad.roll,
                                                              gamepad.pitch,
                                                              gamepad.yaw,
                                                              gamepad.thrust))

            sleep(1 / UPDATE_RATE_HZ)

        except KeyboardInterrupt:
            logging[0] = False


main()
