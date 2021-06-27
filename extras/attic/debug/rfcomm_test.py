#!/usr/bin/env python3
'''
Simple test to send characters to flight controller over Bluetooth

Copyright (C) Simon D. Levy 2021

MIT License
'''

BD_ADDR = '00:06:66:73:E3:E8'
BD_PORT = 1

import bluetooth
from time import sleep

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((BD_ADDR, BD_PORT))

print('connected to ' + BD_ADDR)

k = 0

while True:

        try:

            sock.send(chr(k+97).encode())

            k = (k+1) % 26

            sleep(.01)

        except KeyboardInterrupt:

            sock.close()
            break

