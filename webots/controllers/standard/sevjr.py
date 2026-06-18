#!/usr/bin/env python
'''
   Python flight simulator main for Hackflight with C++ custom physics plugin

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

from time import time

from helper import Helper

def setRudderPosition(left_rudder, right_rudder, position):

    left_rudder.setPosition(position)
    right_rudder.setPosition(position)

def main():

    helper = Helper()

    helper.startMotor('prop_front_left', -1)
    helper.startMotor('prop_front_right', +1)
    helper.startMotor('prop_rear', +1)

    left_rudder = helper.makeMotor('left_rudder')
    right_rudder = helper.makeMotor('right_rudder')

    while True:

        if not helper.step():
            break

        yaw = -helper.cmdinfo[4]

        setRudderPosition(left_rudder, right_rudder, yaw)


main()
