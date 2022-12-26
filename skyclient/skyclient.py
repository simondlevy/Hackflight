#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

from mspparser import MspParser


class SkyParser(MspParser):

    def __init__(self, port):

        MspParser.__init__(self)
        self.port = port

        self.msgVL53L5 = self.serialize_VL53L5_Request()
        self._sendVL53L5()

        self.msgPAA3905 = self.serialize_PAA3905_Request()
        self._sendPAA3905()

    def handle_VL53L5(self, p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34, p41, p42, p43, p44):

        print(p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34, p41, p42, p43, p44)
        self._sendVL53L5()

    def _sendVL53L5(self):

        self.port.write(self.msgVL53L5)

    def handle_PAA3905(self, x, y):

        print(x, y)
        self._sendPAA3905()

    def _sendPAA3905(self):

        self.port.write(self.msgPAA3905)


cmdparser = ArgumentParser()
cmdparser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = cmdparser.parse_args()

port = Serial(args.port, 115200)

skyparser = SkyParser(port)

while True:

    try:

        skyparser.parse(port.read(1))

    except KeyboardInterrupt:

        break
