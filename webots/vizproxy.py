#!/usr/bin/python3

import socket

PORT = 8100

s = socket.socket()

s.connect(('localhost', 8100))

while True:

    print(s.recv(1).decode(), end='')

