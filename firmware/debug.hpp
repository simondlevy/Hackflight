/*
   debug.hpp : Simple serial debugging

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

#include "board.hpp"

namespace hf {

void debug(Board * board, const char * fmt, ...)
{
    va_list ap;       

    va_start(ap, fmt);     

    char buf[1000];

    vsprintf(buf, fmt, ap);

    board->dump(buf);

    va_end(ap);  
}

}
