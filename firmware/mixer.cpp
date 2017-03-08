/*
   mixer.cpp : Mixer class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mixer.c

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

#include "hackflight.hpp"

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

void Mixer::init(class RC * _rc, class Stabilize * _stabilize)
{
    this->stabilize = _stabilize;
    this->rc = _rc;

    // set disarmed motor values
    for (uint8_t i = 0; i < 4; i++)
        this->motorsDisarmed[i] = CONFIG_PWM_MIN;
}

#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 

void Mixer::update(bool armed)
{
    int16_t maxMotor;

    for (uint8_t i = 0; i < 4; i++)
        this->motors[i] = (int16_t)
            (this->rc->command[DEMAND_THROTTLE] * mixerQuadX[i].throttle + 
             this->stabilize->axisPID[AXIS_PITCH] * mixerQuadX[i].pitch + 
             this->stabilize->axisPID[AXIS_ROLL] * mixerQuadX[i].roll - 
             this->stabilize->axisPID[AXIS_YAW] * mixerQuadX[i].yaw);

    maxMotor = this->motors[0];

    for (uint8_t i = 1; i < 4; i++)
        if (this->motors[i] > maxMotor)
            maxMotor = this->motors[i];

    for (uint8_t i = 0; i < 4; i++) {

        if (maxMotor > CONFIG_PWM_MAX)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            this->motors[i] -= maxMotor - CONFIG_PWM_MAX;

        this->motors[i] = constrain(this->motors[i], CONFIG_PWM_MIN, CONFIG_PWM_MAX);

        if (this->rc->throttleIsDown()) {
            this->motors[i] = CONFIG_PWM_MIN;
        } 

        if (!armed) {
            this->motors[i] = motorsDisarmed[i];
        }
    }

    // spin motors
    for (uint8_t i = 0; i < 4; i++)
        Board::writeMotor(i, this->motors[i]);

}
