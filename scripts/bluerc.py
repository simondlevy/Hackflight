#!/usr/bin/env python3

from msppg import MSP_Parser as Parser
import sys
import socket

BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

parser = Parser()
request = parser.serialize_ATTITUDE_Request()

sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
sock.connect((BT_ADDR, BT_PORT))

print('connected to %s' % BT_ADDR)

def handler(c1, c2, c3, c4, c5, c6, c7, c8):

    print(c1, c2, c3, c4, c5, c6, c7, c8)
    sock.send(request)

parser = Parser()
request = parser.serialize_RC_Request()
parser.set_RC_Handler(handler)

sock.send(request)

while True:

    try:

        parser.parse(sock.recv(1))

    except KeyboardInterrupt:

        break

sock.close()
