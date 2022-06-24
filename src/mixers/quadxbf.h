#pragma once

/*
   Mixer values for quad-X frames using Betaflight motor layout:
  
      4cw   2ccw
         \ /
          ^
         / \
      3ccw  1cw
  
   Copyright (C) 2022 Simon D. Levy
  
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
  */

#include "datatypes.h"

// Quad X configuration with Betaflight numbering
static axes_t mixerQuadXbfAxes[] = {
    //  rol   pit    yaw
    { -1.0f, +1.0f, -1.0f },          // REAR_R
    { -1.0f, -1.0f, +1.0f },          // FRONT_R
    { +1.0f, +1.0f, +1.0f },          // REAR_L
    { +1.0f, -1.0f, -1.0f },          // FRONT_L
};

static mixer_t mixerQuadXbf = { 4 , mixerQuadXbfAxes};

static void mixerQuadXbfFun(demands_t * demands, float * motors)
{
    (void)demands;
    (void)motors;
}
