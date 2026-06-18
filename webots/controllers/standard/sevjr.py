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

from newhelper import makeRobot, run, startMotor

def makeRudder(robot, name):

    rudder = robot.getDevice(name)
    rudder.setVelocity(0)

    return rudder

def main():

    robot = makeRobot()

    startMotor(robot, 'prop_front_left', -1)
    startMotor(robot, 'prop_front_right', +1)
    startMotor(robot, 'prop_rear', +1)

    left_rudder = makeRudder(robot, 'left_rudder')
    right_rudder = makeRudder(robot, 'right_rudder')

    run(robot)


main()
