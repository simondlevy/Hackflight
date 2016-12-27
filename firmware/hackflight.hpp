/*
   hackflight.hpp : general header

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

#include <stdint.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>

#include "crossplatform.h"

#ifndef M_PI
#endif

void debug(const char * format, ...);

#include "filters.hpp"
#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "stabilize.hpp"
#include "stabilize_multiwii.hpp"
#include "mixer.hpp"
#include "baro.hpp"
#include "sonars.hpp"
#include "msp.hpp"
#include "hover.hpp"
#include "filters.hpp"

#ifndef abs
#define abs(x)    ((x) > 0 ? (x) : -(x))
#define sgn(x)    ((x) > 0 ? +1 : -1)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 
#endif

#include "config.hpp"
