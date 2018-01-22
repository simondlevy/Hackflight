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

#include "filter.hpp"
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Receiver {

        private: 

            const float margin       = 0.1f;
            const float cyclicExpo   = 0.65f;
            const float cyclicRate   = 0.90f;
            const float throttleMid  = 0.50f;
            const float throttleExpo = 0.20f;

            const float headless     = true;

            static const uint8_t CHANNELS = 8;

            uint8_t commandDelay;     // cycles since most recent movement
            int32_t ppmAverageIndex;  // help with noisy CPPM receivers

            float adjustCommand(float command, uint8_t channel)
            {
                command /= 2;

                if (rawvals[channel] < 0) {
                    command = -command;
                }

                return command;
            }

            float applyCyclicFunction(float command)
            {
                return rcFun(command, cyclicExpo, cyclicRate);
            }

            float makePositiveCommand(uint8_t channel)
            {
                return fabs(rawvals[channel]);
            }

            static float rcFun(float x, float e, float r)
            {
                return (1 + e*(x*x - 1)) * x * r;
            }

            static float throttleFun(float x, float e, float mid)
            {
                float tmp   = x - mid;
                float y = tmp>0 ? 1-mid : (tmp<0 ? mid : 1);
                return mid + tmp*(1-e + e * (tmp*tmp) / (y*y));
            }

        protected: 

            // channel indices
            enum {
                CHANNEL_THROTTLE, // T
                CHANNEL_ROLL,     // A
                CHANNEL_PITCH,    // E
                CHANNEL_YAW,      // R
                CHANNEL_AUX1,
                CHANNEL_AUX2,
                CHANNEL_AUX3,
                CHANNEL_AUX4
            };

            // These must be overridden for each receiver
            virtual void  begin(void) = 0;
            virtual bool  useSerial(void) = 0;
            virtual float readChannel(uint8_t chan) = 0;

            // For logical combinations of stick positions (low, center, high)
            static const uint8_t ROL_LO = (1 << (2 * CHANNEL_ROLL));
            static const uint8_t ROL_CE = (3 << (2 * CHANNEL_ROLL));
            static const uint8_t ROL_HI = (2 << (2 * CHANNEL_ROLL));
            static const uint8_t PIT_LO = (1 << (2 * CHANNEL_PITCH));
            static const uint8_t PIT_CE = (3 << (2 * CHANNEL_PITCH));
            static const uint8_t PIT_HI = (2 << (2 * CHANNEL_PITCH));
            static const uint8_t YAW_LO = (1 << (2 * CHANNEL_YAW));
            static const uint8_t YAW_CE = (3 << (2 * CHANNEL_YAW));
            static const uint8_t YAW_HI = (2 << (2 * CHANNEL_YAW));
            static const uint8_t THR_LO = (1 << (2 * CHANNEL_THROTTLE));
            static const uint8_t THR_CE = (3 << (2 * CHANNEL_THROTTLE));
            static const uint8_t THR_HI = (2 << (2 * CHANNEL_THROTTLE));

            // Stick positions for command combos
            uint8_t sticks;                    

        public:

            float   rawvals[CHANNELS];  // raw [-1,+1] from receiver, for MSP

            demands_t demands;

            // These can be overridden to support various styles of arming (sticks, switches, etc.)

            // Override this if your receiver provides RSSI or other weak-signal detection
            virtual bool lostSignal(void) { return false; }

            virtual bool arming(void)
            {
                return sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE;
            }

            virtual bool disarming(void)
            {
                return sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE;
            }

            virtual uint8_t getAuxState(void) 
            {
                // Auxiliary switch is treated as a fifth axis: 
                // we convert values in interval [-1,+1] to 0, 1, 2
                float aux = rawvals[4];
                return aux < 0 ? 0 : (aux < 0.4 ? 1 : 2);
            }


            void init(void)
            {
                // Do hardware initialization
                begin();

                commandDelay = 0;
                sticks = 0;
                ppmAverageIndex = 0;
            }

            void update(float yawAngle)
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
                    if (rawvals[i] > -1 + margin)
                        stTmp |= 0x80;  // check for MIN
                    if (rawvals[i] < +1 - margin)
                        stTmp |= 0x40;  // check for MAX
                }
                if (stTmp == sticks) {
                    if (commandDelay < 250)
                        commandDelay++;
                } else
                    commandDelay = 0;
                sticks = stTmp;
            
                // Convert raw [-1,+1] to absolute value
                demands.roll  = makePositiveCommand(CHANNEL_ROLL);
                demands.pitch = makePositiveCommand(CHANNEL_PITCH);
                demands.yaw   = makePositiveCommand(CHANNEL_YAW);

                // Apply expo nonlinearity to roll, pitch
                demands.roll  = applyCyclicFunction(demands.roll);
                demands.pitch = applyCyclicFunction(demands.pitch);

                // Put sign back on command, yielding [-0.5,+0.5]
                demands.roll  = adjustCommand(demands.roll, CHANNEL_ROLL);
                demands.pitch = adjustCommand(demands.pitch, CHANNEL_PITCH);
                demands.yaw   = adjustCommand(demands.yaw, CHANNEL_YAW);

                // Support headless mode
                if (headless) {
                    float c = cos(yawAngle);
                    float s = sin(yawAngle);
                    float p = demands.pitch;
                    float r = demands.roll;
                    demands.pitch = c*p + s*r;
                    demands.roll  = c*r - s*p;
                }

                // Yaw demand needs to be reversed
                demands.yaw = -demands.yaw;

                // Special handling for throttle
                float tmp = (rawvals[CHANNEL_THROTTLE] + 1) / 2; // [-1,+1] -> [0,1]
                demands.throttle = throttleFun(tmp, throttleExpo, throttleMid);

            } // computeExpo


            bool changed(void)
            {
                return commandDelay == 20;
            }

             bool throttleIsDown(void)
            {
                return rawvals[CHANNEL_THROTTLE] < -1 + margin;
            }

    }; // class Receiver


} // namespace
