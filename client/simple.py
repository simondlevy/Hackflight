#!/usr/bin/python3

import serial

with serial.Serial('/dev/ttyUSB0', 115200) as ser:

    while True:

        try:

            print('x%02X' % ord(ser.read(1)))

            # print(ser.read(1).decode())

        except KeyboardInterrupt:

            break


