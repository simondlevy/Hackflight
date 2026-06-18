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

from helper import Helper


def makeRudder(helper, name):

    rudder = helper.makeMotor(name)
    rudder.setVelocity(0)
    return rudder


def main():

    helper = Helper()

    helper.startMotor('prop_front_left', -1)
    helper.startMotor('prop_front_right', +1)
    helper.startMotor('prop_rear', +1)

    left_rudder = makeRudder(helper, 'left_rudder')
    right_rudder = makeRudder(helper, 'right_rudder')

    while True:

        if not helper.step():
            break

        yaw = -helper.cmdinfo[4]

        left_rudder.setVelocity(yaw * 4)
        right_rudder.setVelocity(yaw * 4)


main()
