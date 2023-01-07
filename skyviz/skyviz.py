#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

cmdparser = ArgumentParser()
cmdparser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = cmdparser.parse_args()

port = Serial(args.port, 115200)

while True:

    try:

        byte = ord(port.read(1))

        if byte == 0x24:
            print()

        print('x%02x' % byte, end=' ')

    except KeyboardInterrupt:

        break
