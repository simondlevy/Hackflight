#!/usr/bin/python3

from pysticks import get_controller

from mspparser import MspParser

con = get_controller()

while True:

    con.update()

    chanvals = (int(500 * c + 1500) for c in 
             (con.getThrottle(),
             con.getRoll(),
             con.getPitch(),
             con.getYaw(),
             con.getAux(),
             0))

    print(list(chanvals))
