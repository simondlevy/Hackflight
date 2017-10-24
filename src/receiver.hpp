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
    int16_t pitchRollLookupTable[CONFIG_PITCH_LOOKUP_LENGTH];     // lookup table for expo & Receiver rate PITCH+ROLL
    int16_t throttleLookupTable[CONFIG_THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

    RcConfig config;

public:
    void    init(const RcConfig& rcConfig);
    void    update(void);
    int16_t raw[CONFIG_RC_CHANS]; // raw PWM values for MSP
    int16_t command[4];            // stick PWM values for mixer, MSP
    uint8_t sticks;                // stick positions for command combos
    bool    changed(void);
    void    computeExpo(void);
    uint8_t getAuxState(void);
    bool    throttleIsDown(void);

    // Override this if your receiver provides RSSI or other weak-signal detection
    virtual bool     lostSignal(void) { return false; }

};


/********************************************* CPP ********************************************************/

void Receiver::init(const RcConfig& rcConfig)
{
    // Do hardware initialization
    begin();

    memcpy(&config, &rcConfig, sizeof(RcConfig));

    commandDelay = 0;
    sticks = 0;
    averageIndex = 0;

    for (uint8_t i = 0; i < CONFIG_RC_CHANS; i++)
        raw[i] = 1500;

    for (uint8_t i = 0; i < CONFIG_PITCH_LOOKUP_LENGTH; i++)
        pitchRollLookupTable[i] = (2500 + config.expo8 * (i * i - 25)) * i * (int32_t)config.rate8 / 2500;

    for (uint8_t i = 0; i < CONFIG_THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - config.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - config.thrMid8;
        if (tmp < 0)
            y = config.thrMid8;
        throttleLookupTable[i] = 10 * config.thrMid8 + tmp * (100 - config.thrExpo8 + 
            config.thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        throttleLookupTable[i] += 1000;
    }
}

void Receiver::update()
{
    float rawfloat[5];
    float rawDataAverage[5][4];

    // Serial receivers provide clean data and can be read directly
    if (useSerial()) {
        for (uint8_t chan = 0; chan < 5; chan++) {
            rawfloat[chan] = readChannel(chan);
        }
    }

    // Other kinds of receivers require average of channel values to remove noise
    else {
        for (uint8_t chan = 0; chan < 5; chan++) {
            rawDataAverage[chan][averageIndex % 4] = readChannel(chan);
            rawfloat[chan] = 0;
            for (uint8_t i = 0; i < 4; i++)
                rawfloat[chan] += rawDataAverage[chan][i];
            rawfloat[chan] /= 4;
        }
        averageIndex++;
    }

    // For now, convert channel values from [-1,+1] to [1000,2000]
    for (uint8_t chan = 0; chan < 5; chan++) {
        raw[chan] = 1000 + 500*(rawfloat[chan]+1);
    }

    // check stick positions, updating command delay
    uint8_t stTmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (raw[i] > config.mincheck)
            stTmp |= 0x80;  // check for MIN
        if (raw[i] < config.maxcheck)
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
    computeCommand(DEMAND_ROLL);
    computeCommand(DEMAND_PITCH);
    computeCommand(DEMAND_YAW);

    lookupRollPitch(DEMAND_ROLL);
    lookupRollPitch(DEMAND_PITCH);

    adjustCommand(DEMAND_ROLL);
    adjustCommand(DEMAND_PITCH);
    adjustCommand(DEMAND_YAW);

    // Yaw demand needs to be reversed
    command[DEMAND_YAW] = -command[DEMAND_YAW];

    // Special handling for throttle
    int32_t tmp = Filter::constrainMinMax(raw[DEMAND_THROTTLE], config.mincheck, 2000);
    command[DEMAND_THROTTLE] = (uint32_t)(tmp - config.mincheck) * 1000 / (2000 - config.mincheck); // [MINCHECK;2000] -> [0;1000]
    lookupCommand(DEMAND_THROTTLE, throttleLookupTable);

} // computeExp

void Receiver::lookupRollPitch(uint8_t channel)
{
    lookupCommand(channel, pitchRollLookupTable);
}

void Receiver::lookupCommand(uint8_t channel, int16_t * table)
{
    int32_t tmp = command[channel];
    int32_t tmp2 = tmp / 100;
    command[channel] = table[tmp2] + (tmp-tmp2*100) * (table[tmp2+1] - table[tmp2])/100;
}

void Receiver::computeCommand(uint8_t channel)
{
    command[channel] = (std::min)(abs(raw[channel] - 1500), 500);
}

void Receiver::adjustCommand(uint8_t channel)
{
    if (raw[channel] < 1500)
        command[channel] = -command[channel];
}

uint8_t Receiver::getAuxState(void) 
{
    int16_t aux = raw[4];

    return aux < 1500 ? 0 : (aux < 1700 ? 1 : 2);
}

bool Receiver::throttleIsDown(void)
{
    return raw[DEMAND_THROTTLE] < config.mincheck;
}


} // namespace
