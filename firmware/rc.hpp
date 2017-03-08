/*
   rc.hpp : RC receiver class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.h

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

#pragma once

#include <algorithm>
#include <string.h>
//#include "common.hpp"
#include "board.hpp"
#include "config.hpp"


namespace hf {

class RC {
private:
    int16_t dataAverage[CONFIG_RC_CHANS][4];
    uint8_t commandDelay;                               // cycles since most recent movement
    int32_t averageIndex;
    int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
    int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
    int16_t midrc;


public:
    void init();
    void update(Board* _board);

    int16_t data[CONFIG_RC_CHANS]; // raw PWM values for MSP
    int16_t command[4];            // stick PWM values for mixer, MSP
    uint8_t sticks;                // stick positions for command combos
    
    bool changed(void);

    void computeExpo(void);

    uint8_t auxState(void);

    bool throttleIsDown(void);
};


/********************************************* CPP ********************************************************/

inline void RC::init()
{
    midrc = (CONFIG_PWM_MAX + CONFIG_PWM_MIN) / 2;

    memset (dataAverage, 0, 8*4*sizeof(int16_t));

    commandDelay = 0;
    sticks = 0;
    averageIndex = 0;

    for (uint8_t i = 0; i < CONFIG_RC_CHANS; i++)
        data[i] = midrc;

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

inline void RC::update(Board* _board)
{
    if (_board->rcUseSerial()) {
        for (uint8_t chan = 0; chan < 5; chan++) {
            data[chan] = _board->rcReadSerial(chan);
        }
    }

    else {
        for (uint8_t chan = 0; chan < 8; chan++) {

            // get RC PWM
            dataAverage[chan][averageIndex % 4] = _board->readPWM(chan);

            data[chan] = 0;

            for (uint8_t i = 0; i < 4; i++)
                data[chan] += dataAverage[chan][i];
            data[chan] /= 4;
        }

        averageIndex++;
    }

    // check stick positions, updating command delay
    uint8_t stTmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (data[i] > CONFIG_MINCHECK)
            stTmp |= 0x80;  // check for MIN
        if (data[i] < CONFIG_MAXCHECK)
            stTmp |= 0x40;  // check for MAX
    }
    if (stTmp == sticks) {
        if (commandDelay < 250)
            commandDelay++;
    } else
        commandDelay = 0;
    sticks = stTmp;
}

inline bool RC::changed(void)
{
    return commandDelay == 20;
}

#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 

inline void RC::computeExpo(void)
{
    int32_t tmp, tmp2;

    for (uint8_t channel = 0; channel < 3; channel++) {

        tmp = std::min(abs(data[channel] - midrc), 500);

        if (channel != DEMAND_YAW) { // roll, pitch
            tmp2 = tmp / 100;
            command[channel] = 
                lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        } else {                    // yaw
            command[channel] = tmp * -CONFIG_YAW_CONTROL_DIRECTION;
        }

        if (data[channel] < midrc)
            command[channel] = -command[channel];
    }

    tmp = constrain(data[DEMAND_THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    command[DEMAND_THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
        lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [PWM_MIN;PWM_MAX]

} // computeExpo

inline uint8_t RC::auxState(void) 
{
    int16_t aux = data[4];

    return aux < 1500 ? 0 : (aux < 1700 ? 1 : 2);
}

inline bool RC::throttleIsDown(void)
{
    return data[DEMAND_THROTTLE] < CONFIG_MINCHECK;
}


} // namespace
