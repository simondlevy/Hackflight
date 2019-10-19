/* 
   IMU calibration sketch for USFS on STM32L4 Butterfly

   Copyright (C) 2019 Simon D. Levy

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

#include <Wire.h>

#include "UsfsWarmStartAndAccelCal.h"

#include "hackflight.hpp"
#include "boards/arduino/arduino.hpp"

void setup(void)
{
    hf::ArduinoBoard::powerPins(4, 3);

    // Hang a bit 
    delay(100);

    // Start I^2C
    Wire.begin(TWI_PINS_6_7);

    usfs_warm_start_and_accel_cal_setup();
}

void loop(void)
{
    usfs_warm_start_and_accel_cal_loop();
}


