/*
   receiver.hpp : RC receiver class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.h

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstring>
#include <algorithm>

#include "config.hpp"
#include "filter.hpp"

namespace hf {

class Receiver {

protected: 
    // These must be overridden for each receiver
    virtual void  begin(void) = 0;
    virtual bool  useSerial(void) = 0;
    virtual float readChannel(uint8_t chan) = 0;

private:
    void computeCommand(uint8_t channel);
    void adjustCommand(uint8_t channel);
    void lookupRollPitch(uint8_t channel);
    void lookupCommand(uint8_t channel, int16_t * table);

    uint8_t commandDelay;                               // cycles since most recent movement
    int32_t averageIndex;
    int16_t pitchRollLookupTable[CONFIG_PITCHROLL_LOOKUP_LENGTH];     // lookup table for expo & Receiver rate PITCH+ROLL
    int16_t throttleLookupTable[CONFIG_THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

    ReceiverConfig config;

public:
    void    init(const ReceiverConfig& rcConfig);
    void    update(void);
    float   rawvals[CONFIG_RC_CHANS];  // raw [-1,+1] from receiver
    int16_t commands[4];            // stick PWM values 
    float   commandsf[4];
    uint8_t sticks;                // stick positions for command combos
    bool    changed(void);
    void    computeExpo(void);
    uint8_t getAuxState(void);
    bool    throttleIsDown(void);

    int16_t scaleup(float x, float in_min, float in_max, int16_t out_min, int16_t out_max);

    // Override this if your receiver provides RSSI or other weak-signal detection
    virtual bool     lostSignal(void) { return false; }

};


/********************************************* CPP ********************************************************/

void Receiver::init(const ReceiverConfig& rcConfig)
{
    // Do hardware initialization
    begin();

    memcpy(&config, &rcConfig, sizeof(ReceiverConfig));

    commandDelay = 0;
    sticks = 0;
    averageIndex = 0;

    int8_t pitchRollExpo = 100 * config.pitchRollExpo;
    int8_t pitchRollRate = 100 * config.pitchRollRate;
    int8_t throttleMid   = 100 * config.throttleMid;
    int8_t throttleExpo  = 100 * config.throttleExpo;
    
    for (uint8_t i = 0; i < CONFIG_RC_CHANS; i++) 
        rawvals[i] = 0;

    for (uint8_t i = 0; i < CONFIG_PITCHROLL_LOOKUP_LENGTH; i++)
        pitchRollLookupTable[i] = (2500 + pitchRollExpo * (i * i - 25)) * i * (int32_t)pitchRollRate / 2500;

    for (uint8_t i = 0; i < CONFIG_THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - throttleMid;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - throttleMid;
        if (tmp < 0)
            y = throttleMid;
        throttleLookupTable[i] = 10 * throttleMid + tmp * (100 - throttleExpo + throttleExpo * (tmp * tmp) / (y * y)) / 10;
        throttleLookupTable[i] += 1000;
    }
}

void Receiver::update()
{
    float averageRaw[5][4];

    // Serial receivers provide clean data and can be read directly
    if (useSerial()) {
        for (uint8_t chan = 0; chan < 5; chan++) {
            rawvals[chan] = readChannel(chan);
        }
    }

    // Other kinds of receivers require average of channel values to remove noise
    else {
        for (uint8_t chan = 0; chan < 5; chan++) {
            averageRaw[chan][averageIndex % 4] = readChannel(chan);
            rawvals[chan] = 0;
            for (uint8_t i = 0; i < 4; i++)
                rawvals[chan] += averageRaw[chan][i];
            rawvals[chan] /= 4;
        }
        averageIndex++;
    }

    // check stick positions, updating command delay
    uint8_t stTmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (rawvals[i] > -1 + config.margin)
            stTmp |= 0x80;  // check for MIN
        if (rawvals[i] < +1 - config.margin)
            stTmp |= 0x40;  // check for MAX
    }
    if (stTmp == sticks) {
        if (commandDelay < 250)
            commandDelay++;
    } else
        commandDelay = 0;
    sticks = stTmp;
}

bool Receiver::changed(void)
{
    return commandDelay == 20;
}

void Receiver::computeExpo(void)
{
    // Converts raw [-1,+1] to absolute value [0,500]
    computeCommand(DEMAND_ROLL);
    computeCommand(DEMAND_PITCH);
    computeCommand(DEMAND_YAW);

    // Applies nonlinear lookup table to [0,500]
    lookupRollPitch(DEMAND_ROLL);
    lookupRollPitch(DEMAND_PITCH);

    // Puts sign back on command, yielding [-500,+500]
    adjustCommand(DEMAND_ROLL);
    adjustCommand(DEMAND_PITCH);
    adjustCommand(DEMAND_YAW);

    // Yaw demand needs to be reversed
    commands[DEMAND_YAW] = -commands[DEMAND_YAW];

    // Special handling for throttle
    float mincheck = -1 + config.margin;
    float tmp = Filter::constrainMinMaxFloat(rawvals[DEMAND_THROTTLE], mincheck, +1);
    commands[DEMAND_THROTTLE] = (uint32_t)scaleup(tmp, mincheck, +1, 0, 1000);
    lookupCommand(DEMAND_THROTTLE, throttleLookupTable);

} // computeExp

void Receiver::lookupRollPitch(uint8_t channel)
{
    lookupCommand(channel, pitchRollLookupTable);
}

void Receiver::lookupCommand(uint8_t channel, int16_t * table)
{
    int32_t tmp = commands[channel];
    int32_t tmp2 = tmp / 100;
    commands[channel] = table[tmp2] + (tmp-tmp2*100) * (table[tmp2+1] - table[tmp2])/100;
}

void Receiver::computeCommand(uint8_t channel)
{
    int16_t tmp = scaleup(rawvals[channel], -1, +1, 1000, 2000);
    commands[channel] = (std::min)(abs(tmp - 1500), 500);
}

void Receiver::adjustCommand(uint8_t channel)
{
    if (rawvals[channel] < 0)
        commands[channel] = -commands[channel];
}

uint8_t Receiver::getAuxState(void) 
{
    float aux = rawvals[4];

    return aux < 0 ? 0 : (aux < 0.4 ? 1 : 2);
}

bool Receiver::throttleIsDown(void)
{
    return rawvals[DEMAND_THROTTLE] < -1 + config.margin;
}

int16_t Receiver::scaleup(float x, float in_min, float in_max, int16_t out_min, int16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

} // namespace
