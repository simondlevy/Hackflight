#!/usr/bin/python3

import argparse

from mspparser import MspParser

class SkyParser(MspParser):

    def handle_PAA3905(self, x, y):

        print(x, y)

cmdparser = argparse.ArgumentParser()
cmdparser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = cmdparser.parse_args()

print(args.port)

msp = SkyParser()

msg = SkyParser.serialize_PAA3905_Request()

print(msg)
