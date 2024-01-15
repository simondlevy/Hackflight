#!/usr/bin/python3

from serial import Serial
from pysticks import get_controller
from mspparser import MspParser
from time import time

PORT = '/dev/ttyUSB0'

port = Serial(PORT, 115200)

con = get_controller()

prev = 0
count = 0

while True:

    try:

        con.update()

        chanvals = (int(500 * c + 1500) for c in 
                 (con.getThrottle(),
                 con.getRoll(),
                 con.getPitch(),
                 con.getYaw(),
                 con.getAux(),
                 0))

        msg = MspParser.serialize_SET_RAW_RC(*chanvals)

        port.write(msg)

    except KeyboardInterrupt:

        # XXX should send shutdown message

        break

