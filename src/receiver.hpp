/*
   receiver.hpp : RC receiver class header

   Copyright (c) 2018 Simon D. Levy

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
#include <stdint.h>

#include "datatypes.hpp"

namespace hf {

    class Receiver {

        friend class Hackflight;
        friend class RealBoard;
        friend class MspParser;

        private: 

        static constexpr uint8_t DEFAULT_CHANNEL_MAP[6] = {0, 1, 2, 3, 4, 5};

        const float THROTTLE_MARGIN   = 0.1f;
        const float CYCLIC_EXPO       = 0.65f;
        const float CYCLIC_RATE       = 0.90f;
        const float THROTTLE_MID      = 0.00f;
        const float THROTTLE_EXPO     = 0.20f;

        float adjustCommand(float command, uint8_t channel)
        {
            command /= 2;

            if (rawvals[_channelMap[channel]] < 0) {
                command = -command;
            }

            return command;
        }

        float applyCyclicFunction(float command)
        {
            return rcFun(command, CYCLIC_EXPO, CYCLIC_RATE);
        }

        float makePositiveCommand(uint8_t channel)
        {
            return fabs(rawvals[_channelMap[channel]]);
        }

        static float rcFun(float x, float e, float r)
        {
            return (1 + e*(x*x - 1)) * x * r;
        }

        // [-1,+1] -> [0,1] -> [-1,+1] XXX can we simplify?
        float throttleFun(float x)
        {
            float mid = THROTTLE_MID + 0.5;
            float tmp = (x + 1) / 2 - mid;
            float y = tmp>0 ? 1-mid : (tmp<0 ? mid : 1);
            return (mid + tmp*(1-THROTTLE_EXPO + THROTTLE_EXPO * (tmp*tmp) / (y*y))) * 2 - 1;
        }

        protected: 

        static const uint8_t CHANNELS = 7;

        uint8_t _aux1State;
        uint8_t _aux2State;

        // channel indices
        enum {
            CHANNEL_THROTTLE, 
            CHANNEL_ROLL,    
            CHANNEL_PITCH,  
            CHANNEL_YAW,   
            CHANNEL_AUX1,
            CHANNEL_AUX2
        };

        uint8_t _channelMap[6];

        // These must be overridden for each receiver
        virtual bool gotNewFrame(void) = 0;
        virtual void readRawvals(void) = 0;

        // This can be overridden optionally
        virtual void begin(void) { }

        // Software trim
        float _trimRoll;
        float _trimPitch;
        float _trimYaw;

        // Default to non-headless mode
        float headless = false;

        float rawvals[CHANNELS];  // raw [-1,+1] from receiver, for MSP

        demands_t demands;

        float getRawval(uint8_t chan)
        {
            return rawvals[_channelMap[chan]];
        }

        // Override this if your receiver provides RSSI or other weak-signal detection
        virtual bool lostSignal(void) { return false; }

        Receiver(const uint8_t channelMap[6]) // throttle, roll, pitch, yaw, aux, arm
        { 
            memcpy(_channelMap, channelMap, 6);

            _trimRoll  = 0;
            _trimPitch = 0;
            _trimYaw   = 0;
        }

        // Default constructor
        Receiver(void) : Receiver(DEFAULT_CHANNEL_MAP)
        {
        }

         bool getDemands(float yawAngle)
        {
            // Acquire receiver demands, passing yaw angle for headless mode
            // Wait till there's a new frame
            if (!gotNewFrame()) return false;

            // Read raw channel values
            readRawvals();

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

            // Add in software trim
            demands.roll  += _trimRoll;
            demands.pitch += _trimPitch;
            demands.yaw   += _trimYaw;

            // Support headless mode
            if (headless) {
                float c = cos(yawAngle);
                float s = sin(yawAngle);
                float p = demands.pitch;
                float r = demands.roll;
                
                demands.roll  = c*r - s*p;
            }

            // Yaw demand needs to be reversed
            demands.yaw = -demands.yaw;

            // Pass throttle demand through exponential function
            demands.throttle = throttleFun(rawvals[_channelMap[CHANNEL_THROTTLE]]);
            
            // Store auxiliary switch state
            _aux1State = getRawval(CHANNEL_AUX1) >= 0.0 ? (getRawval(CHANNEL_AUX1) > .4 ? 2 : 1) : 0;
            _aux2State = getRawval(CHANNEL_AUX2) >= 0.4 ? 1 : 0;

            // Got a new frame
            return true;

        }  // getDemands

        bool throttleIsDown(void)
        {
            return getRawval(CHANNEL_THROTTLE) < -1 + THROTTLE_MARGIN;
        }

        virtual uint8_t getAux1State(void)
        {
            return _aux1State;
        }

        virtual uint8_t getAux2State(void)
        {
            return _aux2State;
        }

        public:

        static constexpr float STICK_DEADBAND = 0.20;

        void setTrimRoll(float trim)
        {
            _trimRoll = trim;
        }

        void setTrimPitch(float trim)
        {
            _trimPitch = trim;
        }

        void setTrimYaw(float trim)
        {
            _trimYaw = trim;
        }


    }; // class Receiver

} // namespace
