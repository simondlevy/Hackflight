#!/usr/bin/env python3
'''
Communicate with flight controller via Bluetooth

Copyright (C) Simon D. Levy 2021

MIT License
'''

# ADDR = '00:06:66:73:E3:A6'
ADDR = '00:06:66:B8:6E:60'
PORT = 1

import socket
from time import sleep
from sys import stdout

sock = socket.socket(socket.AF_BLUETOOTH,
                     socket.SOCK_STREAM,
                     socket.BTPROTO_RFCOMM)

try:
    sock.connect((ADDR, PORT))

except:
    print('Unable to connect to ' + ADDR)
    exit(1)

print('connected to ' + ADDR)
stdout.flush()

while True:

    try:
        print(sock.recv(1))
        stdout.flush()

    except KeyboardInterrupt:
        break

print('closing ...')
sock.close()
