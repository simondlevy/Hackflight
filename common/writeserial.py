#!/usr/bin/env python

from serial import Serial
from sys import argv

if len(argv) < 2:
    print('Usage:   %s PORTNAME' % argv[0])
    print('Example: %s /dev/ttyUSB0' % argv[0])
    exit(1)

s = Serial(argv[1])

k = 0

while True:

    s.write(chr(ord('a')+k))
    k = (k+1) % 26
