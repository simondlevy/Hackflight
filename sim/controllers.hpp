/*
   controllers.hpp : Common declarations for controller input functions in simulator

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
enum Controller { NONE, TARANIS, SPEKTRUM, EXTREME3D, PS3 };
static Controller controller;

// Downscaling for hypersensitive PS3 controller
static const int PS3_DOWNSCALE = 2;

static int axisdir[5];
static int axismap[5];

