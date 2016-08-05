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

        # pose = <X,Y,Z,THETA>
        self.pose = 0,0,0,0

        self.obstacles = ()

    def update(self, sonars, attitude, altitude):

        '''
        print('Sonars: back: %d cm  front: %d cm left: %d cm right: %d cm' % 
                (sonars[0], sonars[1], sonars[2], sonars[3]))
            
        print('Altitude: %d cm' % (altitude))
        
        print ('Attitude: Pitch: %+3.1f deg   Roll: %+3.1f deg  Heading: %+3.1f deg' % 
                (attitude[0]/10., attitude[1]/10., attitude[2]/10.))
        '''

        self.pose = self.pose[0], self.pose[1], altitude, attitude[2]

        return self.pose, self.obstacles
