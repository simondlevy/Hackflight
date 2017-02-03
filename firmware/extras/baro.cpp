/*
   baro.cpp : Baro class implementation

   Adapted from 

     https://github.com/multiwii/baseflight/blob/master/src/imu.c
     https://github.com/multiwii/baseflight/blob/master/src/sensors.c

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

#ifdef __arm__
extern "C" {
#endif

#include "hackflight.hpp"

void Baro::init(void)
{
    this->historyIdx = 0;
    for (int k=0; k<Baro::TABLE_SIZE; ++k)
        this->historyTable[k] = 0;
    this->pressureSum = 0;
    this->avail = Board::baroInit();
}

bool Baro::available(void)
{
    return this->avail;
}

void Baro::update(void)
{
    // Update hardware
    Board::baroUpdate();

    // Update history table
    int indexplus1 = (this->historyIdx + 1) % Baro::TABLE_SIZE;
    int32_t currentPressure = Board::baroGetPressure();
    this->historyTable[this->historyIdx] = currentPressure;
    this->pressureSum += this->historyTable[this->historyIdx];
    this->pressureSum -= this->historyTable[indexplus1];
    this->historyIdx = indexplus1;
}

int32_t Baro::getAltitude(void)
{
    // Calculate altitude above sea level in cm via baro pressure in Pascals (millibars)
    // See: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    return (int32_t)((1.0f - powf((float)(this->pressureSum / (Baro::TABLE_SIZE - 1)) 
                                        / 101325.0f, 0.190295f)) * 4433000.0f); // XYZ
}

#ifdef __arm__
} // extern "C"
#endif
