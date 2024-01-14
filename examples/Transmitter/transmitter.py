#!/usr/bin/python3

from pysticks import get_controller

con = get_controller()

while True:

    con.update()

    print(('Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   ' +
           'Yaw: %+2.2f   Aux: %+2.2f') %
          (
             con.getThrottle(),
             con.getRoll(),
             con.getPitch(),
             con.getYaw(),
             con.getAux()),
          end='\r')


