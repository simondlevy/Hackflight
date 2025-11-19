/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#define _MAIN

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

void setup() 
{
    hackflight.init(15, false, &Wire1, &SPI, SS, &Serial1);
}

void loop() 
{
    /*
       accel.readSensor();
       gyro.readSensor();

       Serial.print(accel.getAccelX_mss());
       Serial.print("\t");
       Serial.print(accel.getAccelY_mss());
       Serial.print("\t");
       Serial.print(accel.getAccelZ_mss());
       Serial.print("\t");
       Serial.print(gyro.getGyroX_rads());
       Serial.print("\t");
       Serial.print(gyro.getGyroY_rads());
       Serial.print("\t");
       Serial.print(gyro.getGyroZ_rads());
       Serial.print("\t");
       Serial.print(accel.getTemperature_C());
       Serial.print("\n");

       delay(20);
     */
}
