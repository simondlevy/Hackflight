#!/usr/bin/env python3

'''
bluefly.py : Fly your quadcopter from a game controller using Bluetooth + PyGame + MSPPG

**************************** BE CAREFUL ********************************

Copyright (C) Simon D. Levy 2016

This code is free software: you can redistribute it and/or modify
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

# MAC address of your Bluetooth adapter
BT_ADDR = "00:06:66:73:e3:a6"

import socket
import pygame


