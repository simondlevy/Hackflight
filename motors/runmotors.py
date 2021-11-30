#!/usr/bin/env python3

from parser import MspParser
from serial import Serial
from time import sleep
from sys import stdout

class MyParser(MspParser):

    def __init__(self):

        MspParser.__init__(self)

    def handle_ATTITUDE(self, angx, angy, heading):
        pass

PORT = '/dev/ttyS31'

MOTORVAL = 1200

port = Serial(PORT, 115200, timeout=1)

cmd = MspParser.serialize_SET_MOTOR(MOTORVAL, 
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL,
                                    MOTORVAL
                                   )

parser = MyParser()

while True:

    try:
        port.write(cmd)

    except KeyboardInterrupt:
        break

    sleep(.001)

cmd = MspParser.serialize_SET_MOTOR(0, 
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0
                                   )


port.write(cmd)
