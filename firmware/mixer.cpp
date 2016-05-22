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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "mw.hpp"

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

void Mixer::init(Board * board, RC * rc, PID * pid)
{
    this->_board = board;
    this->_pid = pid;
    this->_rc = rc;

    // set disarmed motor values
    for (uint8_t i = 0; i < 4; i++)
        this->motorsDisarmed[i] = CONFIG_MINCOMMAND;
}

void Mixer::update(bool armed)
{
    int16_t maxMotor;
    int16_t motors[4];

    for (uint8_t i = 0; i < 4; i++)
        motors[i] = this->_rc->command[THROTTLE] * mixerQuadX[i].throttle + this->_pid->axisPID[PITCH] * mixerQuadX[i].pitch + 
            this->_pid->axisPID[ROLL] * mixerQuadX[i].roll + -CONFIG_YAW_DIRECTION * this->_pid->axisPID[YAW] * mixerQuadX[i].yaw;

    maxMotor = motors[0];

    for (uint8_t i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];

    for (uint8_t i = 0; i < 4; i++) {

        if (maxMotor > CONFIG_PWM_MAX)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - CONFIG_PWM_MAX;

        motors[i] = constrain(motors[i], CONFIG_PWM_MIN, CONFIG_PWM_MAX);

        if (this->_rc->throttleIsDown()) {
            motors[i] = CONFIG_PWM_MIN;
        } 

        if (!armed) {
            motors[i] = motorsDisarmed[i];
        }
    }

    for (uint8_t i = 0; i < 4; i++)
        this->_board->writeMotor(i, motors[i]);
}

#ifdef __arm__
} // extern "C"
#endif
