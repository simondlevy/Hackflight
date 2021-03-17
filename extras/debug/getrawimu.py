#!/usr/bin/env python3
'''
Uses MSPPG to request and handle raw (integer) IMU messages from flight controller

Copyright (C) Simon D. Levy 2021

MIT License
'''

BAUD = 115200

#PORT = 'COM13'          # Windows
PORT = '/dev/ttyACM0' # Linux

from msppg import Parser, serialize_RAW_IMU_Request
import serial

class RawImuParser(Parser):

    def handle_RAW_IMU(self, ax, ay, az, gx, gy, gz, mx, my, mz):

        print('ax: %+06d  ay: %+06d  az: %+06d | gx: %+06d  gy: %+06d  gz: %+06d ' % (ax, ay, az, gx, gy, gz))
        port.write(request)

if __name__ == '__main__':

    parser = RawImuParser()
    request = serialize_RAW_IMU_Request()
    port = serial.Serial(PORT, BAUD)

    port.write(request)

    while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            break

