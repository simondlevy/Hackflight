#!/usr/bin/python3

import serial

port = serial.Serial('/dev/ttyS0', 115200)

while True:

    try:
        # port.write('a'.encode())
        print(port.read(1))

    except KeyboardInterrupt:
        break
