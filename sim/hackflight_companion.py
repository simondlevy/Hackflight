#!/usr/bin/env python
'''
   hackflight_companion.py : Companion-board Python code

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

import sys
import cv2
from socket_server import serve_socket

# Two command-line arguments: first is camera-client port, second is MSP port
if len(sys.argv) > 2:

    # Serve a socket for camera synching, and a socket for comms
    camera_client = serve_socket(int(sys.argv[1]))
    comms_client  = serve_socket(int(sys.argv[2]))

    while True:

        # Receive the camera sync byte from the client
        camera_client.recv(1)
     
        # Load the image from the temp file
        img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('OpenCV', img)
        cv2.waitKey(1)

        comms_client.send('hello')

# XXX one argument: name of com-port
else:

    None
