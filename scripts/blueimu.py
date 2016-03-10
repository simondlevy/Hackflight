#!/usr/bin/env python3

from msppg import MSP_Parser as Parser

import bluetooth
import sys

BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

parser = Parser()
request = parser.serialize_ATTITUDE_Request()

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
sock.connect((BT_ADDR, BT_PORT))

print('connected to %s' % BT_ADDR)

def handler(pitch, roll, yaw):

    print('%+3.f %+3.f %+3.f' % (pitch, roll, yaw))
    sock.send(request)

parser = Parser()
request = parser.serialize_ATTITUDE_Request()
parser.set_ATTITUDE_Handler(handler)

sock.send(request)

while True:

    try:

        parser.parse(sock.recv(1))

    except KeyboardInterrupt:

        break

sock.close()
