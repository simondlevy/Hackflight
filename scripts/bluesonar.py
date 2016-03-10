#!/usr/bin/env python3

BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

from msppg import MSP_Parser as Parser
from time import sleep
import socket

def handler(baro, sonar):
    print(baro, sonar)
    sock.send(request)

if __name__ == '__main__':

    parser = Parser()
    parser.set_MB1242_Handler(handler)
    request = parser.serialize_MB1242_Request()

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.connect((BT_ADDR, BT_PORT))
    sock.send(request)

    print('connected to %s' % BT_ADDR)

    while True:

        parser.parse(sock.recv(1))
        sleep(.001)
