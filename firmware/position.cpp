/*
   position.cpp : Position-estimation class implementation

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

static const float CONFIG_BARO_CF_ALT = 0.965f;
static const float CONFIG_BARO_CF_VEL = 0.985f;

static bool sonarInRange(void)
{
    return false; // XXX
}


void Position::init(Board * _board, IMU * _imu, Baro * _baro)
{
    this->board = _board;
    this->imu   = _imu;
    this->baro  = _baro;

    this->accelZ = 0;
    this->estAlt = 0;
    this->previousT = 0;
    this->accelVel = 0;
    this->fusedBarosonarAlt = 0;
    this->lastFusedBarosonarAlt = 0;
    this->baroAlt = 0;
    this->baroAltBaseline = 0;
    this->accelAlt = 0;
    this->wasArmed = false;
    this->baroAlt_offset = 0;
    this->sonarTransition = 0;
    this->tiltAngle = 0;
    this->verticalVelocity = 0;
    this->vario = 0;
}

void Position::computeAltitude(bool armed)
{
    this->tiltAngle = max(abs(this->imu->angle[ROLL]), abs(this->imu->angle[PITCH]));

    uint32_t currentT = this->board->getMicros();
    uint32_t dTime = currentT - previousT;

    if (dTime < CONFIG_ALTITUDE_UPDATE_MSEC*1000) 
        return;

    previousT = currentT;

    int32_t baroAltRaw = this->baro->getAltitude();

    // Grab baro baseline on arming
    if (armed) {
        if (!wasArmed) {
            this->baroAltBaseline = baroAltRaw;
            this->accelVel = 0;
            accelAlt = 0;
        }
        this->baroAlt = baroAltRaw - this->baroAltBaseline;
    }
    else {
        this->baroAlt = 0;
    }
    wasArmed = armed;

    // Calculate sonar altitude only if the sonar is facing downwards(<25deg)
    int32_t sonarAlt = -1; // XXX (tiltAngle > 250) ? -1 : sonarAlt * (900.0f - tiltAngle) / 900.0f;

    // Fuse sonarAlt and baroAlt
    if (sonarInRange()) {
        this->baroAlt_offset = this->baroAlt - sonarAlt;
        this->fusedBarosonarAlt = sonarAlt;
    } else {
        this->baroAlt -= this->baroAlt_offset;
        if (sonarAlt > 0) {
            this->sonarTransition = (300 - sonarAlt) / 100.0f;
            this->fusedBarosonarAlt = 
                (int32_t)complementaryFilter((float)sonarAlt, (float)this->baroAlt, (float)this->sonarTransition); 
        }
    }

    // delta acc reading time in seconds
    float dt = this->imu->accelTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    this->accelZ = (float)this->imu->accelSum[2] / (float)this->imu->accelSumCount;
    float vel_acc = this->accelZ * this->imu->accelVelScale * (float)this->imu->accelTimeSum;

    // integrate accelerometer velocity to get vertical velocity
    this->verticalVelocity += vel_acc;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    this->accelAlt += (vel_acc * 0.5f) * dt + this->accelVel * dt;                                         
    this->accelVel += vel_acc;

    // complementary filter for altitude estimation (baro & acc)
    accelAlt = complementaryFilter((float)accelAlt, (float)fusedBarosonarAlt, (float)CONFIG_BARO_CF_ALT);

    //this->estAlt = sonarInRange() ? fusedBarosonarAlt : (int32_t)accelAlt;

    // reset acceleromter sum
    this->imu->resetAccelSum();

    int32_t fusedBaroSonarVel = (int32_t)((fusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime);
    lastFusedBarosonarAlt = fusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = deadbandFilter(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    this->accelVel = complementaryFilter((float)accelVel, (float)fusedBaroSonarVel, (float)CONFIG_BARO_CF_VEL);

    // For now, use baro only for altitude estimate
    this->estAlt = this->baroAlt;
}

#ifdef __arm__
} // extern "C"
#endif
