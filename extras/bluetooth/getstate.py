#!/usr/bin/env python3
'''
Communicate with flight controller via Bluetooth

Copyright (C) Simon D. Levy 2021

MIT License
'''

ADDR = '00:06:66:73:E3:A6'
PORT = 1

import socket
from time import sleep
from sys import stdout

from mspparser import MspParser

class BluetoothMspParser(MspParser):

    def __init__(self, addr, port=1):

        self.sock = socket.socket(socket.AF_BLUETOOTH,
                                  socket.SOCK_STREAM,
                                  socket.BTPROTO_RFCOMM)

        self.addr = addr
        self.port = port

    def handle_STATE(self, _x, _dx, _y, _dy, _z,
                     _dz, phi, _dphi, theta, _dtheta, psi, _dpsi):

        debug('%+3.3f  %+3.3f  %+3.3f' % phi, theta, psi)

        self.send_attitude_request()

    def start(self):

        sock.connect((self.addr, self.port))

        print('connected to ' + self.addr)
        stdout.flush()

        self.send_attitude_request()

    def stop(self):

        self.sock.close()
