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

    this->altHoldChanged = false;
    this->altHoldCorrection = 0;
    this->altHoldValue = 0;
    this->baroAlt = 0;
    this->baroAltBaseline = 0;
    this->errorAltitudeI = 0;
    this->estAlt = 0;
    this->headHold = 0;
    this->initialThrottleHold = 0;
    this->lastBaroAlt = 0;
    this->vario = 0;
    this->wasArmed = false;
}

void Hover::checkSwitch(void)
{
    // If aux switch not in off state
    if (this->rc->auxState() > 0) {

        // If we've just changed state, 
        if (!this->flightMode) {

            // grab altitude and throttle values for hold
            this->altHoldValue = this->estAlt;
            this->initialThrottleHold = this->rc->command[THROTTLE];

            // reset PID values
            this->altHoldPID = 0;
            this->errorAltitudeI = 0;
        }

        this->flightMode = this->rc->auxState() > 1 ? MODE_GUIDED : MODE_ALTHOLD;

    }
    else 
        this->flightMode = MODE_NORMAL;
}

void Hover::updateAltitudePid(bool armed)
{
    uint32_t currentT = Board::getMicros();
    uint32_t dTime = currentT - this->previousT;

    if (dTime < CONFIG_ALTITUDE_UPDATE_MSEC*1000) 
        return;

    this->previousT = currentT;

    // Grab raw baro altitude
    int32_t baroAltRaw = this->baro->getAltitude();

    // Set baro baseline on arming
    if (armed) {

        if (!this->wasArmed) 
            this->baroAltBaseline = baroAltRaw;

        // Compute current baro altitude as offset from baseline
        this->baroAlt = baroAltRaw - this->baroAltBaseline;
        
        // Apply additional LPF to reduce baro noise (faster by 30 µs)
        this->estAlt = (this->estAlt * 6 + this->baroAlt ) >> 3; 

        // Compute altitude velocity based on barometer
        int16_t baroVel = this->estAlt - this->lastBaroAlt;
        this->lastBaroAlt = this->estAlt;
        baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
        baroVel = deadbandFilter(baroVel, 10); // to reduce noise near zero

        // Integrate IMU accelerator Z value to get vario (vertical velocity)
        this->vario += this->imu->computeAccelZ() * dTime;

        // Apply Complimentary Filter to keep the calculated vario based on baro velocity (i.e. near real velocity). 
        // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
        // i.e without delay.
        this->vario = complementaryFilter(this->vario, baroVel, BARO_CF_VEL);

        // PID: P
        int16_t errorAltitudeP = constrain(this->altHoldValue - this->estAlt, -300, 300);
        errorAltitudeP = deadbandFilter(errorAltitudeP, 10); //remove small P param to reduce noise near zero position
        this->altHoldPID = constrain(CONFIG_HOVER_ALT_P * errorAltitudeP >>7, -150, +150);

        // PID: I
        this->errorAltitudeI += CONFIG_HOVER_ALT_I * errorAltitudeP >>6;
        this->errorAltitudeI = constrain(this->errorAltitudeI,-30000,30000);
        this->altHoldPID += errorAltitudeI>>9;    //I in range +/-60

        // PID: D
        this->vario = deadbandFilter(this->vario, 5);
        this->altHoldPID -= constrain(CONFIG_HOVER_ALT_D * this->vario >>4, -150, 150);
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

    if (this->flightMode) { // alt-hold or guided

        // If pilot moved throttle stick signficantly since initiating alt-hold
        if (abs(this->rc->command[THROTTLE]-this->initialThrottleHold) > THROTTLE_NEUTRAL_ZONE) {

            // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 
            // second with cycle time about 3-4ms)
            this->altHoldCorrection += this->rc->command[THROTTLE] - this->initialThrottleHold;
            if(abs(this->altHoldCorrection) > 512) {
                this->altHoldValue += this->altHoldCorrection/512;
                this->altHoldCorrection %= 512;
            }

            this->altHoldChanged = true;
        } 

        // Otherwise, see whether alt-hold just changed
        else if (this->altHoldChanged) {
            this->altHoldValue = this->estAlt;
            this->altHoldChanged = false;

        }

        // Adjust the throttle command via PID to maintain altitude
        this->rc->command[THROTTLE] = this->initialThrottleHold + this->altHoldPID;
    }
} 


#ifdef __arm__
} // extern "C"
#endif
