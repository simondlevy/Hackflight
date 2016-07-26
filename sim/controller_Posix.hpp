/*
   posix.hpp : POSIX (Linux, OS X) API for Hackflight simulator plugin

   Copyright (C) Simon D. Levy, Matt Lubas, and Julio Hidalgo Lopez 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

void posixKbInit(void);

void posixKbGrab(char keys[8]);

void posixKbClose(void);

controller_t posixControllerInit(char * name, const char * ps3name);

void posixControllerGrabAxis(controller_t controller, int * demands, int number, int value);

void posixControllerGrabButton(int * demands, int number);
