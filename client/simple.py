#!/usr/bin/python3

import serial
from time import time

try:
    from __msp__ import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)


'''
class Telemetry(MspParser):

    def __init__(self):

        MspParser.__init__(self)


    def handle_TELEMETRY(self, mode, thrust, roll, pitch, yaw,
                         dx, dy, z, dz, phi, dphi, theta,
                         dtheta, psi, dpsi):

        print(psi)


parser = Telemetry()
'''

prev = 0
count = 0

with serial.Serial('/dev/ttyUSB0', 115200) as ser:

    while True:

        try:

            byte = ord(ser.read(1))

            # print('x%02X' % byte)

            count += 1

            curr = time()

            if curr - prev > 1:

                if prev > 0:
                    print(count)
                    count = 0

                prev = curr

            # parser.parse(ser.read(1))
            # print(ser.read(1).decode())

        except KeyboardInterrupt:

            break


