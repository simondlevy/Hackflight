#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

from mspparser import MspParser


class SkyParser(MspParser):

    def __init__(self, port):

        MspParser.__init__(self)
        self.port = port
        self.msg = self.serialize_PAA3905_Request()
        self._send()

    def _send(self):

        self.port.write(self.msg)

    def handle_PAA3905(self, x, y):

        print(x, y)
        self._send()


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
