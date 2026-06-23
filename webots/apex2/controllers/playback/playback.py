#!/usr/bin/env python
'''
   Hackflight simulator playback

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

from math import cos, sin, acos
from sys import argv

from controller import Supervisor

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None


def euler_to_rotation(phi, theta, psi):
    '''
    https://www.euclideanspace.com/maths/geometry/rotations/conversions/
       eulerToAngle/index.htm
    '''

    c1 = cos(theta/2)
    c2 = cos(psi/2)
    c3 = cos(phi/2)
    s1 = sin(theta/2)
    s2 = sin(psi/2)
    s3 = sin(phi/2)

    return [s1*s2*c3 + c1*c2*s3,
            s1*c2*c3 + c1*s2*s3,
            1 if phi == 0 and theta == 0 and psi == 0 else c1*s2*c3 - s1*c2*s3,
            2 * acos(c1*c2*c3 - s1*s2*s3)]


def start_motor(robot, motor_name, direction):

    motor = robot.getDevice(motor_name)
    motor.setPosition(float('inf'))
    motor.setVelocity(direction * 60)


def show_rangefinder_distances(distances_mm, width, height,
                               dmin_m=.01, dmax_m=4, scaleup=32):

    new_width = width * scaleup
    new_height = height * scaleup

    img = np.zeros((new_height, new_width), dtype=np.uint8)

    for x in range(width):

        for y in range(height):

            d_mm = distances_mm[y * width + x]

            grayval = (255 if d_mm == -1
                       else int((d_mm/1000. - dmin_m) /
                                (dmax_m - dmin_m) * 255))

            cv2.rectangle(img, (x*scaleup, y*scaleup),
                          ((x+1)*scaleup, (y+1)*scaleup),
                          grayval, -1)

    cv2.imshow('rangefinder', img)
    cv2.waitKey(1)


def main():

    robot = Supervisor()

    start_motor(robot, "motor_right_rear_cw", -1)
    start_motor(robot, "motor_right_front_ccw", +1)
    start_motor(robot, "motor_left_rear_ccw", +1)
    start_motor(robot, "motor_left_front_cw", -1)

    timestep = robot.getBasicTimeStep()

    robot_node = robot.getFromDef("robot")

    translation_field = robot_node.getField("frame_translation")

    rotation_field = robot_node.getField("frame_rotation")

    logfile = open(argv[1])

    for line in logfile.readlines()[1:]:  # First line is world name

        if robot.step(int(timestep)) == -1:
            break

        vals = list(map(float, line.split(',')))

        translation_field.setSFVec3f([vals[1], -vals[3], vals[5]])

        rotation_field.setSFRotation(
                euler_to_rotation(vals[7], vals[9], -vals[11]))

        rangefinder_distances = vals[6:]

        if len(rangefinder_distances) > 0:
            if cv2 is None:
                print(rangefinder_distances)
            else:
                show_rangefinder_distances(rangefinder_distances,
                        len(rangefinder_distances), 1)


main()
