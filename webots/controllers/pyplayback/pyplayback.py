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

from controller import Supervisor


class DiyQuad(Supervisor):

    ROBOT_NAME = "diyquad"

    def __init__(self):

        Supervisor.__init__(self)

    def getFromDef(self):

        return Supervisor.getFromDef(self, self.ROBOT_NAME)


def euler_to_rotation(euler):
    '''
    https://www.euclideanspace.com/maths/geometry/rotations/conversions/
       eulerToAngle/index.htm
    '''

    phi = euler[0]
    theta = euler[1]
    psi = euler[2]

    c1 = cos(theta/2)
    c2 = cos(psi/2)
    c3 = cos(phi/2)
    s1 = sin(theta/2)
    s2 = sin(psi/2)
    s3 = sin(phi/2)

    return [s1*s2*c3 + c1*c2*s3,
            s1*c2*c3 + c1*s2*s3,
            1 if phi==0 and theta==0 and psi==0 else c1*s2*c3 - s1*c2*s3,
            2 * acos(c1*c2*c3 - s1*s2*s3)]


def main():

    quad = DiyQuad()

    timestep = quad.getBasicTimeStep()

    robot_node = quad.getFromDef()

    translation_field = robot_node.getField("translation")

    rotation_field = robot_node.getField("rotation")

    logfile = open('../../../simtest/log.csv')

    for line in logfile.readlines():

        if quad.step(int(timestep)) == -1:
            break

        vals = list(map(float, line.split(',')))

        xyz = vals[:3]

        translation_field.setSFVec3f(xyz)

        euler = vals[3:6]

        rotation_field.setSFRotation(euler_to_rotation(euler))

        rangefinder_distances = vals[6:]


main()
