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

from controller import Supervisor

class DiyQuad(Supervisor):

    ROBOT_NAME = "diyquad"

    def __init__(self):

        Supervisor.__init__(self)

    def getFromDef(self):

        return Supervisor.getFromDef(self, self.ROBOT_NAME)

def main():

    quad = DiyQuad()
    
    timestep = quad.getBasicTimeStep();

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

        rotation = vals[3:6]

        rangefinder_distances = vals[6:]


main()
