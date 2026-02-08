#!/usr/bin/env python
'''
   Python flight simulator main for Hackflight with C++ custom physics plugin

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

from sys import argv

MODE_IDLE =  0
MODE_ARMED = 1
MODE_HOVERING = 2
MODE_AUTONOMOUS = 3
MODE_LANDING = 4
MODE_PANIC = 5

def main():


    setpointlogfp = open(argv[3], 'w')

    mode = MODE_IDLE
