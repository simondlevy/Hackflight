#!/usr/bin/env python3

from parser import MspParser
from serial import Serial
from time import sleep

PORT = '/dev/ttyS31'


class MyParser(MspParser):

    def __init__(self, port):

        MspParser.__init__(self)

        self.port = port

    def handle_ATTITUDE(self, angx, angy, heading):
        pass

    def set_motors(self, m1val, m2val, m3val, m4val):

        cmd = self.serialize_SET_MOTOR(m1val, m2val, m3val, m4val,
                                       1000, 1000, 1000, 1000,
                                       1000, 1000, 1000, 1000,
                                       1000, 1000, 1000, 1000)

        self.port.write(cmd)


MOTORVAL = 1200

port = Serial(PORT, 115200, timeout=1)

parser = MyParser(port)

while True:

    try:

        parser.set_motors(1000, MOTORVAL, 1000, MOTORVAL)

    except KeyboardInterrupt:
        break

    sleep(.001)

parser.set_motors(1000, 1000, 1000, 1000)
