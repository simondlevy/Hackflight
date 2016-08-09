#!/usr/bin/env python3

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

    def __init__(self, sensor_angles=(0,), min_distance_cm=0, max_distance_cm=1000):

        # Store sensor parameters for later
        self.sensor_angles = sensor_angles
        self.sensor_min_distance_cm = min_distance_cm
        self.sensor_max_distance_cm = max_distance_cm

        # For now, just 2D: pose = <x,y,theta>
        self.pose = 0,0,0

        # No obstacles yet
        self.obstacles = ()

    def update(self, sonars, attitude, altitude):

        print('Sonars: back: %d cm  front: %d cm left: %d cm right: %d cm' % 
                (sonars[0], sonars[1], sonars[2], sonars[3]))
            
        '''
        print('Altitude: %d cm' % (altitude))
        
        print ('Attitude: Pitch: %+3.1f deg   Roll: %+3.1f deg  Heading: %+3.1f deg' % 
                (attitude[0]/10., attitude[1]/10., attitude[2]/10.))
        '''

        self.pose = self.pose[0], self.pose[1], attitude[2]

        return self.pose, self.obstacles
