/*
   linux.hpp: Hackflight SimBoard class implementation for Linux

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

#include "sim.hpp"

#include <stdio.h>

void hf::SimBoard::cputime(struct timespec * tv)
{
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, tv);
}

void hf::Board::outbuf(char * buf, const char * fmt, va_list ap)
{
    vsprintf(buf, fmt, ap); 
    printf("%s", buf); 
}

