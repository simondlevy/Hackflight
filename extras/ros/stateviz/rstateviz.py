#!/usr/bin/env python
'''
rstateviz.py: ROS rviz script for visualizing quadcopter state in 3D space

Copyright (C) 2018 Simon D. Levy

Adapted from 

  https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/basic_controls.py

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

MARKER_COLOR    = 1.0, 0.0, 0.0
MARKER_RESOURCE = 'package://stateviz/arrowhead.stl'
MARKER_SCALE    = .02

import rospy
import copy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler

from math import sin, pi

server = None
br = None
counter = 0
marker = None

def frameCallback(msg):

    global counter, br, marker
    time = rospy.Time.now()

    cycle = sin(counter/140.0)

    roll = 0
    pitch = 0
    yaw = cycle * pi
    rotation_quaternion = quaternion_from_euler(roll, pitch, yaw)
   
    x = 0
    y = 0
    z = 0 #cycle*2.0
    translation = (x, y, z)

    br.sendTransform( translation, rotation_quaternion, time, 'map', 'moving_frame')                
    counter += 1

def processFeedback(feedback):

    server.applyChanges()

def normalizeQuaternion(orientation):

    norm = orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2
    s = norm**(-0.5)
    orientation.x *= s
    orientation.y *= s
    orientation.z *= s
    orientation.w *= s

def makeVehicleMarker(position):

    marker = InteractiveMarker()
    marker.header.frame_id = 'moving_frame'
    marker.pose.position = position
    marker.scale = 1

    marker.name = 'quadcopter'

    control =  InteractiveMarkerControl()
    control.always_visible = True

    meshMarker = Marker()
    meshMarker.type = Marker.MESH_RESOURCE
    meshMarker.mesh_resource = MARKER_RESOURCE
    meshMarker.scale.x, meshMarker.scale.y, meshMarker.scale.z = (tuple([marker.scale*MARKER_SCALE]))*3
    meshMarker.color.r, meshMarker.color.g, meshMarker.color.b = MARKER_COLOR
    meshMarker.color.a = 1.0

    control.markers.append(meshMarker)
    marker.controls.append(control)
 
    control = InteractiveMarkerControl()
    q = quaternion_from_euler(0, 0, 0)
    control.orientation.x = q[0]
    control.orientation.y = q[1]
    control.orientation.z = q[2] 
    control.orientation.w = q[3]

    normalizeQuaternion(control.orientation)

    marker.controls.append(copy.deepcopy(control))
    marker.controls.append(control)

    server.insert(marker, processFeedback)

if __name__=='__main__':

    rospy.init_node('basic_controls')

    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer('basic_controls')

    position = Point(0, 0, 0)
    makeVehicleMarker(position)

    server.applyChanges()

    rospy.spin()
