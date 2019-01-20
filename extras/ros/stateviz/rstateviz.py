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

NODE_NAME = 'stateviz'

MARKER_COLOR    = 1.0, 0.0, 0.0
MARKER_RESOURCE = 'package://stateviz/arrowhead.stl'
MARKER_START    = 0.08, 0.08, 0.
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
vehicleMarker = None
vehicleControl = None

def normalizeQuaternion(orientation):

    norm = orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2
    s = norm**(-0.5)
    orientation.x *= s
    orientation.y *= s
    orientation.z *= s
    orientation.w *= s

def frameCallback(msg):

    global counter, br, vehicleMarker, vehicleControl
    time = rospy.Time.now()

    inc = counter/140.0

    roll = 0
    pitch = 0
    yaw = inc
    rotation = quaternion_from_euler(roll, pitch, yaw)

    x = sin(inc) * 2.0
    y = 0 
    z = 0 
    translation = (x, y, z)

    br.sendTransform(translation, rotation, time, 'vehicle_frame', 'map')                

    counter += 1
    
def processFeedback(feedback):

    server.applyChanges()

if __name__=='__main__':

    rospy.init_node(NODE_NAME)
    br = TransformBroadcaster()
    rospy.Timer(rospy.Duration(0.01), frameCallback)
    server = InteractiveMarkerServer(NODE_NAME)

    vehicleMarker = InteractiveMarker()
    vehicleMarker.header.frame_id = 'vehicle_frame'
    vehicleMarker.pose.position = Point(*MARKER_START)
    vehicleMarker.scale = 1
    vehicleMarker.name = 'quadcopter'
    q = quaternion_from_euler(0, 0, 0)
    vehicleMarker.pose.orientation.x = q[0]
    vehicleMarker.pose.orientation.y = q[1]
    vehicleMarker.pose.orientation.z = q[2] 
    vehicleMarker.pose.orientation.w = q[3]
    normalizeQuaternion(vehicleMarker.pose.orientation)

    vehicleMesh = Marker()
    vehicleMesh.type = Marker.MESH_RESOURCE
    vehicleMesh.mesh_resource = MARKER_RESOURCE
    vehicleMesh.scale.x, vehicleMesh.scale.y, vehicleMesh.scale.z = (tuple([vehicleMarker.scale*MARKER_SCALE]))*3
    vehicleMesh.color.r, vehicleMesh.color.g, vehicleMesh.color.b = MARKER_COLOR
    vehicleMesh.color.a = 1.0

    vehicleControl =  InteractiveMarkerControl()
    vehicleControl.always_visible = True
    vehicleControl.markers.append(vehicleMesh)

    vehicleMarker.controls.append(vehicleControl)

    server.insert(vehicleMarker, processFeedback)

    server.applyChanges()

    rospy.spin()
