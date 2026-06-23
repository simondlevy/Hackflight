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


def main():

    helper = Helper()

    helper.startMotor('motor_right_rear_cw', Helper.PROP_CW)
    helper.startMotor('motor_right_front_ccw', Helper.PROP_CCW)
    helper.startMotor('motor_left_rear_ccw', Helper.PROP_CCW)
    helper.startMotor('motor_left_front_cw', Helper.PROP_CW)

    while True:
        if not helper.step():
            break


main()
