#!/usr/bin/env python3

'''
setrc.py Uses MSPPG to set raw RC values in flight controller.  Makes the vehicle pitch forward
when Channel 5 (three-position switch) is down.  BE CAREFUL!!!

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

# Standard for Baseflight
BAUD            = 115200

# Tune these for safety / performance
UPDATE_RATE_HZ  = 200
WAIT_TIME_SEC   = .1
PWM_MIN         = 1000
PWM_MAX         = 2000

# Values for outgoing message
CHANNEL_NEUTRAL       = 1500
CHANNEL_AUTOPILOT     = 1600

from msppg import MSP_Parser as Parser, serialize_SET_RAW_RC, serialize_RC_Request
import serial
import time
from sys import argv
import threading

if len(argv) < 2:

    print('Usage: python3 %s PORT' % argv[0])
    print('Example: python3 %s /dev/ttyUSB0' % argv[0])
    exit(1)

parser = Parser()
port = serial.Serial(argv[1], BAUD)

# This thread sethe throttle based on incoming messages
class SetterThread(threading.Thread):

    def __init__(self, getter):

        threading.Thread.__init__(self, target=self.setter)

        self.setDaemon(True)

        self.getter = getter

    def setter(self):

        while(True):

            getter = self.getter

            if getter.autopilot: 

                # Make the vehicle pitch forward on autopilot
                message = serialize_SET_RAW_RC(getter.c1, CHANNEL_AUTOPILOT, getter.c3, getter.c4, 
                            CHANNEL_NEUTRAL, 0, 0, 0)
                port.write(message)

            time.sleep(1./UPDATE_RATE_HZ)

class Getter:

    def __init__(self):

        self.c5prev = 0

        self.request = serialize_RC_Request()

        port.write(self.request)

        parser.set_RC_Handler(self.get)

        self.autopilot = False
        self.offtime = 0
        self.timestart = time.time()

    def _error(self, msg):

        print(msg)
        exit(1)

    def get(self, c1, c2, c3, c4, c5, c6, c7, c8):

        # Store stick values
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4

        # Disallow startup with switch down
        if self.offtime == 0 and c5 > 1000:
            self._error('Please turn off switch before starting')

        # Check arming

        # Switch moved down
        if c5 > PWM_MIN and self.c5prev < PWM_MIN and self.offtime > WAIT_TIME_SEC:

            self.autopilot = True
            self.throttle = 0
            self.throttledir = +1

        # Switch moved back up
        if c5 < PWM_MIN and self.c5prev > PWM_MIN:

            self.autopilot = False

            self.timestart = time.time()

        if not self.autopilot:

            self.offtime = time.time() - self.timestart

        self.c5prev = c5

        port.write(self.request)

getter = Getter()

setter = SetterThread(getter)

setter.start()

while True:

    try:

        parser.parse(port.read(1))

    except KeyboardInterrupt:

        break
