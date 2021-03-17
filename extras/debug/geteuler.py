#!/usr/bin/env python3
'''
Uses MSPPG to request and handle attitude messages from flight controller

Copyright (C) Simon D. Levy 2021

MIT License
'''

BAUD = 115200

#PORT = 'COM22'          # Windows
PORT = '/dev/ttyACM1' # Linux

from msppg import Parser, serialize_ATTITUDE_RADIANS_Request
import serial
from time import sleep
from sys import stdout

class AttitudeParser(Parser):

    def handle_ATTITUDE_RADIANS(self, roll, pitch, yaw):

        print('%+3.3f %+3.3f %+3.3f' % (roll, pitch, yaw))
        stdout.flush()
        port.write(request)

if __name__ == '__main__':

    parser = AttitudeParser()
    request = serialize_ATTITUDE_RADIANS_Request()
    port = serial.Serial(PORT, BAUD)

    # Connecting causes reboot on ESP32
    sleep(1)

    port.write(request)

    while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            break

