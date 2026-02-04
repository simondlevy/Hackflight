#!/usr/bin/env python3

'''
Copyright (C) 2026 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

# Map size, scale
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32

from roboviz import MapVisualizer

import numpy as np
from sys import argv, exit
from time import sleep

'''
def threadfunc(robot, slam, timestamps, lidars, odometries, mapbytes, pose):
    Threaded function runs SLAM, setting the map bytes and robot pose for display
    on the main thread.

    # Initialize time for delay
    prevtime = 0

    # Loop over scans    
    for scanno in range(len(lidars)):

        if odometries is None:
                  
             # Update SLAM with lidar alone
            slam.update(lidars[scanno])

        else:
        
            # Convert odometry to velocities
            velocities = robot.computePoseChange(odometries[scanno])

            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)

        # Get new position
        pose[0],pose[1],pose[2] = slam.getpos()    

        # Get new map
        slam.getmap(mapbytes)

        # Add delay to yield to main thread
        currtime = timestamps[scanno] / 1.e6 # Convert usec to sec
        if prevtime > 0:
            sleep(currtime-prevtime)
        prevtime = currtime
'''
    
def main():
	    
    if len(argv) < 2:
        print('Usage:   %s LOGFILE' % argv[0])
        exit(1)
    
    data = np.loadtxt(argv[1], delimiter=',')

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    for row in data:

        pose = row[0], row[1], row[5]

        print(pose)

    '''
    # Loop forever,displaying current map and pose
    while True:

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(pose[0]/1000., pose[1]/1000., pose[2], mapbytes):
            exit(0)
    '''
                    
main()
