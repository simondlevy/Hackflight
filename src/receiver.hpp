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
#include <cmath>

#include "config.hpp"
#include "filter.hpp"
#include "debug.hpp"

namespace hf {

class Receiver {

public:

protected: 

    // These must be overridden for each receiver
    virtual void  begin(void) = 0;
    virtual bool  useSerial(void) = 0;
    virtual float readChannel(uint8_t chan) = 0;

    // For logical combinations of stick positions (low, center, high)
    static const uint8_t ROL_LO = (1 << (2 * DEMAND_ROLL));
    static const uint8_t ROL_CE = (3 << (2 * DEMAND_ROLL));
    static const uint8_t ROL_HI = (2 << (2 * DEMAND_ROLL));
    static const uint8_t PIT_LO = (1 << (2 * DEMAND_PITCH));
    static const uint8_t PIT_CE = (3 << (2 * DEMAND_PITCH));
    static const uint8_t PIT_HI = (2 << (2 * DEMAND_PITCH));
    static const uint8_t YAW_LO = (1 << (2 * DEMAND_YAW));
    static const uint8_t YAW_CE = (3 << (2 * DEMAND_YAW));
    static const uint8_t YAW_HI = (2 << (2 * DEMAND_YAW));
    static const uint8_t THR_LO = (1 << (2 * DEMAND_THROTTLE));
    static const uint8_t THR_CE = (3 << (2 * DEMAND_THROTTLE));
    static const uint8_t THR_HI = (2 << (2 * DEMAND_THROTTLE));

    // Stick positions for command combos
    uint8_t sticks;                    

private:

    float adjustCommand(float command, uint8_t channel);

    float applyPitchRollFunction(float command);
    float makePositiveCommand(uint8_t channel);

    static float rcFun(float x, float e, float r);
    static float throttleFun(float x, float e, float m);

    uint8_t commandDelay;     // cycles since most recent movement
    int32_t ppmAverageIndex;  // help with noisy CPPM receivers

    ReceiverConfig config;

public:

    // These can be overridden to support various styles of arming (sticks, switches, etc.) and
    // auxiliary-state implementations (switches, buttons, etc.)
    virtual bool    arming(void);
    virtual bool    disarming(void);
    virtual uint8_t getAuxState(void);

    float   rawvals[ReceiverConfig::CHANNELS];  // raw [-1,+1] from receiver, for MSP

    float   demandThrottle;
    float   demandRoll;
    float   demandPitch;
    float   demandYaw;

    void    init(const ReceiverConfig& rcConfig);
    void    update(void);
    bool    changed(void);
    void    computeExpo(float yawAngle);
    bool    throttleIsDown(void);

    // Override this if your receiver provides RSSI or other weak-signal detection
    virtual bool lostSignal(void) { return false; }
};


/********************************************* CPP ********************************************************/

// arming(), disarming(), getAuxState() can be overridden as needed --------------------

bool Receiver::arming(void)
{
    return sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE;
}

bool Receiver::disarming(void)
{
    return sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE;
}

uint8_t Receiver::getAuxState(void) 
{
    float aux = rawvals[4];

    return aux < 0 ? 0 : (aux < 0.4 ? 1 : 2);
}

// --------------------------------------------------------------------------------------

void Receiver::init(const ReceiverConfig& rcConfig)
{
    // Do hardware initialization
    begin();

    memcpy(&config, &rcConfig, sizeof(ReceiverConfig));

    commandDelay = 0;
    sticks = 0;
    ppmAverageIndex = 0;
}

void Receiver::update()
{
    float averageRaw[5][4];

    // Serial receivers provide clean data and can be read directly
    if (useSerial()) {
        for (uint8_t chan = 0; chan < 5; chan++) {
            rawvals[chan] = readChannel(chan);
            //Debug::printf("%d:%f%c", chan, rawvals[chan], chan==4?'\n':'\t');
        }
    }

    // Other kinds of receivers require average of channel values to remove noise
    else {
        for (uint8_t chan = 0; chan < 5; chan++) {
            averageRaw[chan][ppmAverageIndex % 4] = readChannel(chan);
            rawvals[chan] = 0;
            for (uint8_t i = 0; i < 4; i++)
                rawvals[chan] += averageRaw[chan][i];
            rawvals[chan] /= 4;
        }
        ppmAverageIndex++;
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

void Receiver::computeExpo(float yawAngle)
{
    // Convert raw [-1,+1] to absolute value
    demandRoll  = makePositiveCommand(DEMAND_ROLL);
    demandPitch = makePositiveCommand(DEMAND_PITCH);
    demandYaw   = makePositiveCommand(DEMAND_YAW);

    // Apply expo nonlinearity to roll, pitch
    demandRoll  = applyPitchRollFunction(demandRoll);
    demandPitch = applyPitchRollFunction(demandPitch);

    // Put sign back on command, yielding [-0.5,+0.5]
    demandRoll  = adjustCommand(demandRoll, DEMAND_ROLL);
    demandPitch = adjustCommand(demandPitch, DEMAND_PITCH);
    demandYaw   = adjustCommand(demandYaw, DEMAND_YAW);

    // Yaw demand needs to be reversed
    demandYaw = -demandYaw;

    // Special handling for throttle
    float tmp = (rawvals[DEMAND_THROTTLE] + 1) / 2; // [-1,+1] -> [0,1]
    demandThrottle = throttleFun(tmp, config.throttleExpo, config.throttleMid);

} // computeExp

float Receiver::makePositiveCommand(uint8_t channel)
{
    return fabs(rawvals[channel]);
}

float Receiver::applyPitchRollFunction(float command)
{
    return rcFun(command, config.pitchRollExpo, config.pitchRollRate);
}

float Receiver::rcFun(float x, float e, float r)
{
    return (1 + e*(x*x - 1)) * x * r;
}

float Receiver::adjustCommand(float command, uint8_t channel)
{
    command /= 2;

    if (rawvals[channel] < 0) {
        command = -command;
    }

    return command;
}

float Receiver::throttleFun(float x, float e, float mid)
{
    float tmp   = x - mid;
    float y = tmp>0 ? 1-mid : (tmp<0 ? mid : 1);
    return mid + tmp*(1-e + e * (tmp*tmp) / (y*y));
}

bool Receiver::throttleIsDown(void)
{
    return rawvals[DEMAND_THROTTLE] < -1 + config.margin;
}

} // namespace
