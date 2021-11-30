#!/usr/bin/env python3

from parser import MspParser
from serial import Serial
from time import sleep
from sys import stdout

class MyParser(MspParser):

    def __init__(self, port, cmd):

        MspParser.__init__(self)

        self.port = port
        self.cmd = cmd

    def handle_ATTITUDE(self, angx, angy, heading):
        print(angx, angy, heading)
        self.port.write(self.cmd)

# PORT = 'COM31'
PORT = '/dev/ttyS31'

port = Serial(PORT, 115200, timeout=1)

cmd = MspParser.serialize_ATTITUDE_Request()

parser = MyParser(port, cmd)

port.write(cmd)

while True:

    try:
        byte = port.read(1)

    except KeyboardInterrupt:
        break

    parser.parse(byte)
