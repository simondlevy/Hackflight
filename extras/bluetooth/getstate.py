#!/usr/bin/env python3
'''
Communicate with flight controller via Bluetooth

Copyright (C) Simon D. Levy 2021

MIT License
'''

import socket
from sys import stdout
from time import sleep

from mspparser import MspParser


class BluetoothMspParser(MspParser):

    def __init__(self, addr, port=1):

        self.sock = socket.socket(socket.AF_BLUETOOTH,
                                  socket.SOCK_STREAM,
                                  socket.BTPROTO_RFCOMM)

        self.addr = addr
        self.port = port

        self.state_request = MspParser.serialize_STATE_Request()

    def handle_STATE(self, _x, _dx, _y, _dy, _z,
                     _dz, phi, _dphi, theta, _dtheta, psi, _dpsi):

        BluetoothMspParser._debug('%+3.3f  %+3.3f  %+3.3f' % phi, theta, psi)

        self.send_state_request()

    def handle_ACTUATOR_TYPE(self, atype):
        return

    def handle_RECEIVER(self, c1, c2, c3, c4, c5, c6):
        return

    @staticmethod
    def _debug(msg):
        print(msg)
        stdout.flush()

    def _send_state_request(self):

        self.sock.send(self.state_request)

    def start(self):

        self.sock.connect((self.addr, self.port))

        BluetoothMspParser._debug('connected to ' + self.addr)

        self._send_state_request()

    def stop(self):

        BluetoothMspParser._debug('Shutting down ...')

        self.sock.close()


def main():

    btp = BluetoothMspParser('00:06:66:73:E3:A6')

    btp.start()

    while True:

        try:
            sleep(.001)

        except KeyboardInterrupt:
            break

    btp.stop()


main()
