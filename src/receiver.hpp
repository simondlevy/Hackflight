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

#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Receiver {

        friend class Hackflight;
        friend class RealBoard;
        friend class MSP;

        private: 

        const float MARGIN            = 0.1f;
        const float CYCLIC_EXPO       = 0.65f;
        const float CYCLIC_RATE       = 0.90f;
        const float THROTTLE_MID      = 0.00f;
        const float THROTTLE_EXPO     = 0.20f;

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
            return rcFun(command, CYCLIC_EXPO, CYCLIC_RATE);
        }

        float makePositiveCommand(uint8_t channel)
        {
            return fabs(rawvals[channel]);
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

        static const uint8_t CHANNELS = 5;

        uint8_t _auxState;

        // channel indices
        enum {
            CHANNEL_THROTTLE, // T
            CHANNEL_ROLL,     // A
            CHANNEL_PITCH,    // E
            CHANNEL_YAW,      // R
            CHANNEL_AUX

        };

        // These must be overridden for each receiver
        virtual void  begin(void) = 0;
        virtual bool  gotNewFrame(void) = 0;
        virtual void  readRawvals(void) = 0;

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

        // Software trim
        float _trimRoll;
        float _trimPitch;
        float _trimYaw;

        // Default to non-headless mode
        float headless = false;

        float rawvals[CHANNELS];  // raw [-1,+1] from receiver, for MSP

        demands_t demands;

        // These can be overridden to support various styles of arming (sticks, switches, etc.)

        // Override this if your receiver provides RSSI or other weak-signal detection
        virtual bool lostSignal(void) { return false; }

        Receiver(float trimRoll=0, float trimPitch=0, float trimYaw=0) : 
            _trimRoll(trimRoll), _trimPitch(trimPitch), _trimYaw(trimYaw) { }

        virtual bool arming(void)
        {
            return sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE;
        }

        virtual bool disarming(void)
        {
            return sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE;
        }

        void init(void)
        {
            // Do hardware initialization
            begin();

            sticks = 0;
        }

        bool getDemands(float yawAngle)
        {
            // Wait till there's a new frame
            if (!gotNewFrame()) return false;

            // Read raw channel values
            readRawvals();

            // Check stick positions, updating command delay
            uint8_t stTmp = 0;
            for (uint8_t i = 0; i < 4; i++) {
                stTmp >>= 2;
                if (rawvals[i] > -1 + MARGIN)
                    stTmp |= 0x80;  // check for MIN
                if (rawvals[i] < +1 - MARGIN)
                    stTmp |= 0x40;  // check for MAX
            }

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
            demands.throttle = throttleFun(rawvals[CHANNEL_THROTTLE]);
            
            // Store auxiliary switch state 
            _auxState = rawvals[CHANNEL_AUX] >= 0.0 ? (rawvals[CHANNEL_AUX] > .4 ? 2 : 1) : 0;

            // Got a new frame
            return true;

        }  // getDemands


        bool throttleIsDown(void)
        {
            return rawvals[CHANNEL_THROTTLE] < -1 + MARGIN;
        }

        virtual uint8_t getAuxState(void)
        {
            return _auxState;
        }


        public:

        static constexpr float STICK_DEADBAND = 0.20;

    }; // class Receiver

} // namespace
