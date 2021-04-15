/*
   Abstract RC receiver class

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

#include <stdint.h>
#include <math.h>

#include "demands.hpp"

namespace hf {

    class Receiver {

        friend class Hackflight;
        friend class SerialTask;
        friend class PidTask;

        private: 

            const float THROTTLE_MARGIN = 0.1f;
            const float CYCLIC_EXPO     = 0.65f;
            const float CYCLIC_RATE     = 0.90f;
            const float THROTTLE_EXPO   = 0.20f;
            const float AUX_THRESHOLD   = 0.4f;

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

            // [-1,+1] -> [0,1] -> [-1,+1]
            float throttleFun(float x)
            {
                float mid = 0.5;
                float tmp = (x + 1) / 2 - mid;
                float y = tmp>0 ? 1-mid : (tmp<0 ? mid : 1);
                return (mid + tmp*(1-THROTTLE_EXPO + THROTTLE_EXPO * (tmp*tmp) / (y*y))) * 2 - 1;
            }

        protected: 

            // maximum number of channels that any receiver will send (of which we'll use six)
            static const uint8_t MAXCHAN = 8;

            uint8_t _aux1State = 0;
            uint8_t _aux2State = 0;

            float _demandScale = 0;

            // channel indices
            enum {
                CHANNEL_THROTTLE, 
                CHANNEL_ROLL,    
                CHANNEL_PITCH,  
                CHANNEL_YAW,   
                CHANNEL_AUX1,
                CHANNEL_AUX2
            };

            uint8_t _channelMap[6] = {0};

            // These must be overridden for each receiver
            virtual bool gotNewFrame(void) = 0;
            virtual void readRawvals(void) = 0;

            // This can be overridden optionally
            virtual void begin(void) { }

            // Software trim
            float _trimRoll = 0;
            float _trimPitch = 0;
            float _trimYaw = 0;

            // Default to non-headless mode
            float headless = false;

            // Raw receiver values in [-1,+1]
            float rawvals[MAXCHAN] = {};  

            float demands[4] = {}; // Throttle, Roll, Pitch, Yaw

            float getRawval(uint8_t chan)
            {
                return rawvals[_channelMap[chan]];
            }

            // Override this if your receiver provides RSSI or other weak-signal detection
            virtual bool lostSignal(void) { return false; }

            /**
              * channelMap: throttle, roll, pitch, yaw, aux, arm
              */
            Receiver(const uint8_t channelMap[6], float demandScale=1.0) 
            { 
                for (uint8_t k=0; k<6; ++k) {
                    _channelMap[k] = channelMap[k];
                }

                _trimRoll  = 0;
                _trimPitch = 0;
                _trimYaw   = 0;

                _demandScale = demandScale;
            }

            bool getDemands(void)
            {
                // Wait till there's a new frame
                if (!gotNewFrame()) return false;

                // Read raw channel values
                readRawvals();

                // Convert raw [-1,+1] to absolute value
                demands[DEMANDS_ROLL]  = makePositiveCommand(CHANNEL_ROLL);
                demands[DEMANDS_PITCH] = makePositiveCommand(CHANNEL_PITCH);
                demands[DEMANDS_YAW]   = makePositiveCommand(CHANNEL_YAW);

                // Apply expo nonlinearity to roll, pitch
                demands[DEMANDS_ROLL]  = applyCyclicFunction(demands[DEMANDS_ROLL]);
                demands[DEMANDS_PITCH] = applyCyclicFunction(demands[DEMANDS_PITCH]);

                // Put sign back on command, yielding [-0.5,+0.5]
                demands[DEMANDS_ROLL]  = adjustCommand(demands[DEMANDS_ROLL], CHANNEL_ROLL);
                demands[DEMANDS_PITCH] = adjustCommand(demands[DEMANDS_PITCH], CHANNEL_PITCH);
                demands[DEMANDS_YAW]   = adjustCommand(demands[DEMANDS_YAW], CHANNEL_YAW);

                // Add in software trim
                demands[DEMANDS_ROLL]  += _trimRoll;
                demands[DEMANDS_PITCH] += _trimPitch;
                demands[DEMANDS_YAW]   += _trimYaw;

                // Negate pitch demand, so that pulling back on stick means positive demand.
                // Doing this keeps demands consistent with Euler angles (positive pitch = nose up).
                demands[DEMANDS_PITCH] = -demands[DEMANDS_PITCH];

                // Pass throttle demand through exponential function
                demands[DEMANDS_THROTTLE] = throttleFun(rawvals[_channelMap[CHANNEL_THROTTLE]]);

                // Store auxiliary switch state
                _aux1State = getRawval(CHANNEL_AUX1) >= 0.0 ? (getRawval(CHANNEL_AUX1) > AUX_THRESHOLD ? 2 : 1) : 0;
                _aux2State = getRawval(CHANNEL_AUX2) >= AUX_THRESHOLD ? 1 : 0;

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
