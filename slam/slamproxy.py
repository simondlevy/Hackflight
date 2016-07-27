#!/usr/bin/env python3

'''
   slamproxy.py : phony SLAM data provider for testing 3DSLAM visualiztion

   Allow us to test our SLAM visualizer without running a flight simulator
   or actual vehicle.

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
'''

CUBESIZE = 0.1

import socket
import time
import sys

import msppg

if __name__ == '__main__':

    host =     sys.argv[1]  if len(sys.argv) > 1 else 'localhost'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 20000

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try :
        sock.connect((host, port))

    except:
        print('Connection resfused: make sure visualization server is running')
        exit(1)

    # Create an MSP parser to handle pose message requests
    parser = msppg.MSP_Parser()

    '''
    for count in range(20):

            try:
                sock.send(('%f,%f,%f\n' % (x,y,z)).encode('utf-8'))
            except:
                break

            x += CUBESIZE

            time.sleep(1)
    '''
