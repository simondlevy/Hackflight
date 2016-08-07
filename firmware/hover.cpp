/*
   hover.cpp : Hover-in-place code

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

#include <math.h>

#ifdef __arm__
extern "C" {
#endif

#include "mw.hpp"
#include "pidvals.hpp"

void Hover::init(IMU * _imu, Baro * _baro, RC * _rc)
{
    this->imu   = _imu;
    this->baro  = _baro;
    this->rc = _rc;

    this->estAlt = 0;
    this->vario = 0;
    this->headHold = 0;
    this->baroAlt = 0;
    this->baroAltBaseline = 0;
    this->wasArmed = false;
}

void Hover::checkSwitch(void)
{
    // If aux switch not in off state
    if (this->rc->auxState() > 0) {

        // If we've just changed state, grab altitude for hold
        if (!this->flightMode) 
            this->altHoldValue = this->estAlt;

        this->flightMode = this->rc->auxState() > 1 ? MODE_GUIDED : MODE_ALTHOLD;

    }
    else 
        this->flightMode = MODE_NORMAL;
}

void Hover::updateAltitudePid(bool armed)
{
    // Grab raw baro altitude
    int32_t baroAltRaw = this->baro->getAltitude();

    // Set baro baseline on arming
    if (armed) {

        if (!this->wasArmed) 
            this->baroAltBaseline = baroAltRaw;

        // Compute current baro altitude as offset from baseline
        this->baroAlt = baroAltRaw - this->baroAltBaseline;

        printf("%d\n", this->baroAlt);
    }

    // If not armed, reset baro altitude
    else 
        this->baroAlt = 0;
    
    // Track arming status to detect change
    this->wasArmed = armed;
}


void Hover::perform(void)
{
    // For now, support navigation tasks in simulation only
#ifndef _SIM
    return;
#endif

} 


#ifdef __arm__
} // extern "C"
#endif
