/*
   crossplatform.h : support for compiling firmware on Windows

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


#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

#ifdef _WIN32
#define SPRINTF sprintf_s
#define STRCPY  strcpy_s
#define VSNPRINTF vsnprintf_s
#define lrintf(x) (float)(int)((x)+0.5)
#else
#define SPRINTF sprintf
#define STRCPY  strcpy
#define VSNPRINTF vsnprintf
#include <stdbool.h>
#endif

#ifndef M_PIf
#define M_PIf static_cast<float>(3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PI
#define M_PI static_cast<double>(3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PIl
#define M_PIl static_cast<long double>(3.1415926535897932384626433832795028841972)
#endif

#define constrain(val, lo, hi) (val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val))
