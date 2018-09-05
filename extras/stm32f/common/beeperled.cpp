/*
   beeperled.cpp : code for boards that use beeper signals to control LED

   Copyright (C) 2018 Simon D. Levy 

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

#include "beeperled.h"

// Here we put code that interacts with Cleanflight
extern "C" {

#include "drivers/sound_beeper.h"
#include "pg/beeper_dev.h"

    void beeperLedInit(void)
    {
        // Set up the LED (uses the beeper for some reason)
        beeperInit(beeperDevConfig());

        // Turn it off
        systemBeep(true);
    }

    void beeperLedSet(bool isOn)
    {
        systemBeep(!isOn);
    }
}
