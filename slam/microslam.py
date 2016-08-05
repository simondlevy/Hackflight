'''
microslam.py : minimal 3D SLAM algorithm

Copyright (C) Simon D. Levy, Matt Lubas, Alfredo Rwagaju 2016

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

class MicroSLAM(object):

    def __init__(self):

        self.pose = 0,0,0
        self.obstacles = ()

    def update(self, sonars, attitude, altitude):

        return

    def getPose(self):

        return self.pose

    def getObstacles(self):

        return self.obstacles

