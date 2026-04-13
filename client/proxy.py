#!/usr/bin/python3

'''
Copyright (C) 2026 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import serial
from time import sleep


if __name__ == '__main__':


    try:
        port = serial.Serial('/dev/ttyUSB0', 115200)

    except serial.SerialException:
        print('Unable to open port ' + args.port)
        exit(0)

    while True:

        try:

            msg = 'A'
            port.write(msg.encode())
            sleep(.005)

        except KeyboardInterrupt:
            break
            

