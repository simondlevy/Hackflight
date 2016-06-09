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

#include "mw.hpp"
#include <math.h>

void Baro::init(Board * board) 
{
    this->_board = board;

    this->histIdx = 0;
    for (int k=0; k<Baro::TABLE_SIZE_MAX; ++k)
        this->histTable[k] = 0;
    this->pressureSum = 0;

    this->avail = board->baroInit();
}

bool Baro::available(void)
{
    return this->avail;
}

int32_t Baro::getAltitude(void)
{
    int indexplus1 = (this->histIdx + 1) % Baro::TABLE_SIZE;
    this->histTable[this->histIdx] = this->_board->baroGetPressure();
    this->pressureSum += this->histTable[this->histIdx];
    this->pressureSum -= this->histTable[indexplus1];
    this->histIdx = indexplus1;

    // Calculates height from ground in cm via baro pressure
    // See: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    return lrintf((1.0f - powf((float)(this->pressureSum / (Baro::TABLE_SIZE - 1)) /
       101325.0f, 0.190295f)) * 4433000.0f);

}

#ifdef __arm__
} // extern "C"
#endif
