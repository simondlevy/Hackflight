#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

cmdparser = ArgumentParser()
cmdparser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = cmdparser.parse_args()

port = Serial(args.port, 115200)

while True:

    try:

        print('x%02x' % ord(port.read(1)))

    except KeyboardInterrupt:

        break
