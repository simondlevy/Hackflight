/*
   Abstract RC receiver class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "HF_demands.hpp"

extern bool copilot_receiverGotNewFrame;
extern bool copilot_receiverLostSignal;
extern float copilot_receiverThrottle;
extern float copilot_receiverRoll;
extern float copilot_receiverPitch;
extern float copilot_receiverYaw;
extern float copilot_receiverAux1;
extern float copilot_receiverAux2;

namespace hf {

    class Receiver {

        friend class ClosedLoopTask;
        friend class HackflightPure;
        friend class HackflightFull;
        friend class SerialTask;
        friend class PidTask;

        private: 

            const float THROTTLE_MARGIN = 0.1f;
            const float CYCLIC_EXPO     = 0.65f;
            const float CYCLIC_RATE     = 0.90f;
            const float THROTTLE_EXPO   = 0.20f;
            const float AUX_THRESHOLD   = 0.4f;

            float _demandScale = 0;

            void adjustCommand(uint8_t index, float rawval)
            {
                _demands[index] /= 2;

                if (rawval < 0) {
                    _demands[index] = -_demands[index];
                }
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

            // Demands (throttle, roll, pitch, yaw)
            float _demands[4] = {};

        public:

            Receiver(const float demandScale, const float trim[3] = NULL) 
            { 
                if (trim) {
                    _trimRoll  = trim[0];
                    _trimPitch = trim[1];
                    _trimYaw   = trim[2];
                }

                _demandScale = demandScale;
            }

            void update(void)
            {
                // Wait till there's a new frame
                if (copilot_receiverGotNewFrame) return;

                // Convert raw [-1,+1] to absolute value
                _demands[DEMANDS_ROLL]  = fabs(copilot_receiverRoll);
                _demands[DEMANDS_PITCH] = fabs(copilot_receiverPitch);
                _demands[DEMANDS_YAW]   = fabs(copilot_receiverYaw);

                // Apply expo nonlinearity to roll, pitch
                _demands[DEMANDS_ROLL]  = applyCyclicFunction(_demands[DEMANDS_ROLL]);
                _demands[DEMANDS_PITCH] = applyCyclicFunction(_demands[DEMANDS_PITCH]);

                // Put sign back on command, yielding [-0.5,+0.5]
                adjustCommand(DEMANDS_ROLL, copilot_receiverRoll);
                adjustCommand(DEMANDS_PITCH, copilot_receiverPitch);
                adjustCommand(DEMANDS_YAW, copilot_receiverYaw);

                // Add in software trim
                _demands[DEMANDS_ROLL]  += _trimRoll;
                _demands[DEMANDS_PITCH] += _trimPitch;
                _demands[DEMANDS_YAW]   += _trimYaw;

                // Pass throttle demand through exponential function
                _demands[DEMANDS_THROTTLE] = throttleFun(copilot_receiverThrottle);

                // Multiply by demand scale
                _demands[DEMANDS_ROLL] *= _demandScale;
                _demands[DEMANDS_PITCH] *= _demandScale;
                _demands[DEMANDS_YAW] *= _demandScale;

            }  // update

            static const uint8_t MAX_DEMANDS = 10; // arbitrary

            bool inArmedState(void)
            {
                return copilot_receiverAux1 > 0;
            }

            void getDemands(float * demands)
            {
                memcpy(demands, _demands, sizeof(_demands));
            }

            bool inactive(void)
            {
                return copilot_receiverThrottle < (-1 + THROTTLE_MARGIN);
            }

    }; // class Receiver

} // namespace
