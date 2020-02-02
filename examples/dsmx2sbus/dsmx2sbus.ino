/*
   Test DSMX => SBUS translation

   Copyright (c) 2020 Simon D. Levy

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

#include <SBUS.h>
#include <DSMRX.h>

SBUS sbus = SBUS(Serial1);

void setup(void)
{
    sbus.begin();
}

void loop(void)
{
      static float chanvals[16];

      chanvals[0] = -1;

      chanvals[5] = millis() > 3000 ? +1 : -1;

      sbus.writeCal(chanvals);

      delay(10);
}

