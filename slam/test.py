#!/usr/bin/env python3

'''

Simple test of MicroSLAM: simulated square room with single forward-facing sensor, 
steadily rotating vehicle.

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

from microslam import MicroSLAM
from slamvis2d import TwoDSlamVis

slam = MicroSLAM()

disp = TwoDSlamVis()
