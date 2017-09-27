'''
setup.py : script for building Hackflight GCS executable via py2exe

Copyright (C) Simon D. Levy 2017

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
from distutils.core import setup
'''

from distutils.core import setup
from subprocess import call
import py2exe
from os import system

system('mkdir dist')
system('mkdir dist\media')
system('copy media\icon.xbm dist\media')
system('copy media\hackflight.ico dist\media')
system('copy media\*.gif dist\media')

setup(windows = ['hackflight.py'])

#system('move dist hackflight-gcs')
