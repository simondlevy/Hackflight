/*
   Abstract RC receiver class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "stream_receiver.h"

#include "HF_demands.hpp"

namespace hf {

    class Receiver {

        friend class ClosedLoopTask;
        friend class HackflightPure;
        friend class HackflightFull;
        friend class SerialComms;
        friend class PidTask;

        private: 

            const float THROTTLE_MARGIN = 0.1f;
            const float CYCLIC_EXPO     = 0.65f;
            const float CYCLIC_RATE     = 0.90f;
            const float THROTTLE_EXPO   = 0.20f;
            const float AUX_THRESHOLD   = 0.4f;

            float _demandScale = 0;

            float adjustCommand(float demand, float rawval)
            {
                demand /= 2;

                demand = rawval < 0 ? -demand : demand;

                return demand;
            }

            float applyCyclicFunction(float command)
            {
                return rcFun(command, CYCLIC_EXPO, CYCLIC_RATE);
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

            // Software trim
            float _trimRoll = 0;
            float _trimPitch = 0;
            float _trimYaw = 0;

        public:

            Receiver(const float demandScale, const float trim[3] = NULL) 
            { 
                _trimRoll  = trim ? trim[0] : 0;
                _trimPitch = trim ? trim[1] : 0;
                _trimYaw   = trim ? trim[2] : 0;

                _demandScale = demandScale;
            }

            void getDemands(demands_t & demands)
            {
                // Pass throttle demand through exponential function
                demands.throttle = throttleFun(stream_receiverThrottle);

                // Convert raw [-1,+1] to absolute value
                // Apply expo nonlinearity to roll, pitch
                // Put sign back on command, yielding [-0.5,+0.5]
                // Add in software trim
                // Multiply by demand scale

                
                demands.roll   = (adjustCommand(applyCyclicFunction(fabs(stream_receiverRoll)), stream_receiverRoll)   + _trimRoll) *_demandScale;
                demands.pitch  = (adjustCommand(applyCyclicFunction(fabs(stream_receiverPitch)), stream_receiverPitch) + _trimPitch) *_demandScale;

                demands.yaw  = fabs(stream_receiverYaw);
                demands.yaw   = adjustCommand(demands.yaw,   stream_receiverYaw);
                demands.yaw   += _trimYaw;
                demands.yaw   *= _demandScale;

            } // getDemands

            static const uint8_t MAX_DEMANDS = 10; // arbitrary

            bool inArmedState(void)
            {
                return stream_receiverAux1 > 0;
            }

            bool inactive(void)
            {
                return stream_receiverThrottle < (-1 + THROTTLE_MARGIN);
            }

    }; // class Receiver

} // namespace
