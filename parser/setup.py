'''
Python distutils setup file for installing MSPPG

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

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

from distutils.core import setup

setup (name = 'msppg',
       packages = ['msppg_resources'],
       scripts = ['msppg.py'],
       package_data={'msppg_resources' : ['*']},
       version = '0.1',
       description = 'Multiwii Serial Protocol Parser Generator',
       author='Rob Jones, Chris Lavin, Blake Liebling, Alec Singer, Simon D. Levy',
       author_email='simon.d.levy@gmail.com',
       url='https://github.com/simondlevy/MSPPG',
       license='LGPL',
       platforms='Linux; Windows; OS X')

