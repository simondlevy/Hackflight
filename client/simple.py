#!/usr/bin/python3

import serial
from time import time

try:
    from __msp__ import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)


class Telemetry(MspParser):

    def __init__(self):
        MspParser.__init__(self)
        self._count = 0


    def handle_TELEMETRY(self, mode, thrust, roll, pitch, yaw,
                         dx, dy, z, dz, phi, dphi, theta,
                         dtheta, psi, dpsi):
        print(psi)

    def handle_BOGUS(self, psi):
        print(self._count, psi)
        self._count += 1


parser = Telemetry()

with serial.Serial('/dev/ttyUSB0', 115200) as ser:

    while True:

        try:

            parser.parse(ser.read(1))

        except KeyboardInterrupt:

            break


