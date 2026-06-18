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

STARTUP_YAW_TIME = 0.25
STARTUP_YAW_VELOCITY = -0.5

def main():

    helper = Helper()

    helper.startMotor('prop_front_left', -1)
    helper.startMotor('prop_front_right', +1)
    helper.startMotor('prop_rear', +1)

    left_rudder = helper.makeMotor('left_rudder')
    right_rudder = helper.makeMotor('right_rudder')

    time_start = time()

    dt = 1 / helper.timestep

    while True:

        if not helper.step():
            break

        time_curr = time()

        time_prev = time_curr

        print('%3.3f' % dt)

        yaw_velocity = (STARTUP_YAW_VELOCITY
               if time_curr - time_start < STARTUP_YAW_TIME
               else -helper.cmdinfo[4])

        left_rudder.setVelocity(yaw_velocity * 4)
        right_rudder.setVelocity(yaw_velocity * 4)


main()
