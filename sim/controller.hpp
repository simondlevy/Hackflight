/*
   controller.hpp : Common declarations for controller input functions in simulator

   Each OS handles controllers differently.

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

// We currently support these controllers
enum controller_t { KEYBOARD, TARANIS, SPEKTRUM, EXTREME3D, PS3 , XBOX360 };

controller_t controllerInit(void);
void         controllerRead(controller_t controller, float * demands);
void         controllerClose(void);

// in v_repExtHackflight.cpp
extern void kbRespond(char key, char * keys);
