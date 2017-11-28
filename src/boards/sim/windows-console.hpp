/*
   windows-console.hpp: Debugging support for Windows console programs

   Copyright (C) Simon D. Levy 2017

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

#include "windows.hpp"

#include <stdio.h>
#include <varargs.h>

void hf::Board::outbuf(char * buf, const char * fmt, va_list ap)
{
    vsprintf_s(buf, fmt, ap); 
    printf("%s", buf); 
}
