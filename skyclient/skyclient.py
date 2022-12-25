#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

from mspparser import MspParser

class SkyParser(MspParser):

    def handle_PAA3905(self, x, y):

        print(x, y)

cmdparser = ArgumentParser()
cmdparser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = cmdparser.parse_args()

msp = SkyParser()

msg = SkyParser.serialize_PAA3905_Request()

port = Serial(args.port, 115200)

port.write(msg)

while True:

    try:

        byte = port.read(1)

        print('x%02X' % ord(byte))

    except KeyboardInterrupt:

        break
