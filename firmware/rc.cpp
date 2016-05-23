/*
   rc.cpp : RC receiver class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.c

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

#include <strings.h>

void RC::init(Board * board)
{
    this->_board = board;

    this->midrc = (CONFIG_PWM_MAX + CONFIG_PWM_MIN) / 2;

    bzero (this->dataAverage, 8*4*sizeof(int16_t));

    this->commandDelay = 0;
    this->sticks = 0;
    this->averageIndex = 0;

    for (uint8_t i = 0; i < CONFIG_RC_CHANS; i++)
        this->data[i] = this->midrc;

    for (uint8_t i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + CONFIG_RC_EXPO_8 * (i * i - 25)) * i * (int32_t)CONFIG_RC_RATE_8 / 2500;

    for (uint8_t i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - CONFIG_THR_MID_8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - CONFIG_THR_MID_8;
        if (tmp < 0)
            y = CONFIG_THR_MID_8;
        lookupThrottleRC[i] = 10 * CONFIG_THR_MID_8 + tmp * (100 - CONFIG_THR_EXPO_8 + 
                (int32_t)CONFIG_THR_EXPO_8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = CONFIG_PWM_MIN + (int32_t)(CONFIG_PWM_MAX - CONFIG_PWM_MIN) * 
            lookupThrottleRC[i] / 1000; // [PWM_MIN;PWM_MAX]
    }
}

void RC::update(void)
{
    for (uint8_t chan = 0; chan < 8; chan++) {
    
        // get RC PWM
        this->dataAverage[chan][this->averageIndex % 4] = 
            this->_board->readPWM(CONFIG_RCMAP[chan]);

        this->data[chan] = 0;

        for (uint8_t i = 0; i < 4; i++)
            this->data[chan] += this->dataAverage[chan][i];
        this->data[chan] /= 4;
    }

    this->averageIndex++;

    // check stick positions, updating command delay
    uint8_t stTmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
        //printf("%d ", this->data[i]);
        stTmp >>= 2;
        if (this->data[i] > CONFIG_MINCHECK)
            stTmp |= 0x80;  // check for MIN
        if (this->data[i] < CONFIG_MAXCHECK)
            stTmp |= 0x40;  // check for MAX
    }
    //printf("\n");
    if (stTmp == this->sticks) {
        if (this->commandDelay < 250)
            this->commandDelay++;
    } else
        this->commandDelay = 0;
    this->sticks = stTmp;
}

bool RC::changed(void)
{
  return this->commandDelay == 20;
}

void RC::computeExpo(void)
{
    int32_t tmp, tmp2;

    for (uint8_t axis = 0; axis < 3; axis++) {

        tmp = min(abs(this->data[axis] - this->midrc), 500);

        if (axis != 2) {        // ROLL & PITCH
            tmp2 = tmp / 100;
            this->command[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        } else {                // YAW
            this->command[axis] = tmp * -CONFIG_YAW_CONTROL_DIRECTION;
        }

        if (this->data[axis] < this->midrc)
            this->command[axis] = -this->command[axis];
    }

    tmp = constrain(this->data[THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    this->command[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
            lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [PWM_MIN;PWM_MAX]

} // computeExpo

uint8_t RC::auxState(void) 
{
    int16_t aux = this->data[4];

    return aux < 1500 ? 0 : (aux < 1700 ? 1 : 2);
}

bool RC::throttleIsDown(void)
{
    return this->data[THROTTLE] < CONFIG_MINCHECK;
}

#ifdef __arm__
} // extern "C"
#endif
