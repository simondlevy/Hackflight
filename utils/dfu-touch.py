#!/bin/python3
'''
Puts Arduino-compatible device into DFU mode for flashing

https://forum.arduino.cc/t/reset-nano-every-via-1200-baud-touch/939949
'''

import serial
import time
import sys

BAUD = 1200
SLEEP = 1

port = sys.argv[1]

print('Performing %d-bps touch reset on serial port %s' % (BAUD, port))

com = serial.Serial(port, BAUD, dsrdtr=True)

com.dtr = True

com.write('0000'.encode())

time.sleep(SLEEP)

com.dtr = False

time.sleep(SLEEP)

com.close()
