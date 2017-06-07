/*
   sim_extras.hpp : Extra simulator functionality

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

#include "vrepsimboard.hpp"
#include <hackflight.hpp>
#include <board.hpp>
#include <stdio.h>

void simExtrasStart(void)
{
}

void simExtrasUpdate(void)
{
}

void simExtrasMessage(int message, int * auxiliaryData, void * customData)
{
}

void simExtrasStop(void)
{
}

namespace hf {

uint8_t VrepSimBoard::serialAvailableBytes(void)
{
    return 0;
}

uint8_t VrepSimBoard::serialReadByte(void) 
{
    return 0;
}

void VrepSimBoard::serialWriteByte(uint8_t c)
{
    (void)c;
}

} // namespace hf
