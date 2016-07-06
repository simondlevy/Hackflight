#!/usr/bin/env python
'''
   hackflight_companion.py : Companion-board Python code.  Runs in 
   Python2 instead of Python3, so we can install OpenCV without 
   major hassles.

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
import numpy as np

from msppg import MSP_Parser

def processImage(image):

    # Blur image to remove noise
    frame = cv2.GaussianBlur(image, (3, 3), 0)

    # Switch image from BGR colorspace to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    blueMin = (100,  50,  10)
    blueMax = (255, 255, 255)

    # Find where image in range
    mask = cv2.inRange(hsv, blueMin, blueMax)

    # Find centroid of mask
    x, y = np.where(mask)
    if len(x) / float(np.prod(mask.shape)) > 0.2:
        x,y = np.int(np.mean(x)), np.int(np.mean(y))
        cv2.putText(image, 'WATER', (y,x), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

if __name__ == '__main__':

    # Create a parser for MSP messaging
    parser = MSP_Parser()

    # More than two command-line arguments means simulation mode.  First arg is camera-client port, 
    # second is MSP port, third is input image file name, fourth is outpt image file name.
    if len(sys.argv) > 2:

        from socket_server import serve_socket

        # Serve a socket for camera synching, and a socket for comms
        camera_client = serve_socket(int(sys.argv[1]))
        comms_client  = serve_socket(int(sys.argv[2]))
        image_from_sim_name  = sys.argv[3]
        image_to_sim_name  = sys.argv[4]

        while True:

            # Receive the camera sync byte from the client
            camera_client.recv(1)
         
            # Load the image from the temp file
            image = cv2.imread(image_from_sim_name, cv2.IMREAD_COLOR)

            # Process it
            mask = processImage(image)

            # Write the processed image to a file for the simulator to display
            cv2.imwrite(image_to_sim_name, image)

            comms_client.send('hello')

    # Fewer than three arguments: live mode or camera-test mode
    else:

        commport = sys.arg[1] if len(sys.argv) > 1 else None

        cap = cv2.VideoCapture(0)

        while True:

            success, image = cap.read()

            if success:

                # Process image
                processImage(image) 

                # Test mode; display image
                if commport is None:
                    cv2.imshow('OpenCV', image)
                    if cv2.waitKey(1) == 27:  # ESC
                        break


