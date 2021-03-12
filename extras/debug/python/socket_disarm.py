#!/usr/bin/env python3
'''
Test script for MSP disarming of board over a socket

Copyright (C) Simon D. Levy, Pep Marti, Juan Gallostra Acin 2021

MIT License
'''

HOST = '137.113.118.68' # thales.cs.wlu.edu
PORT = 20000

from msppg import serialize_SET_ARMED
import socket

if __name__ == "__main__":

    sock = socket.socket()

    print('Attempting to connect to server %s:%d' % (HOST, PORT))

    try:
        sock.connect((HOST, PORT)) # Note tuple!
        print('Connected!')
    except socket.error:
        print('Failed to connect')
        exit(1)

    try:
        sock.send(serialize_SET_ARMED(False))
    except socket.error:
        print('Failed to send')
