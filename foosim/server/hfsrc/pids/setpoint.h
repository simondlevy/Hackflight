/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "pid.h"

class SetPointPid {
    
    private:

        static bool inBand(const float value, const float band) 
        {
            return value > -band && value < band;
        }

        static float constrainAbs(const float v, const float lim)
        {
            return v < -lim ? -lim : v > +lim ? +lim : v;
        }

        bool inBandPrev;
        float errorI;

    public:

        void modifyDemand(
                const float k_p,
                const float k_i,
                const float k_stick_deadband,
                const float k_pilot_vel_max,
                const float k_windup_max,
                const float velocity,
                const float target,
                const bool reset,
                const float center,
                float & demand,
                bool & movedIntoBand 
                ) 
        {
            // Is stick demand in deadband?
            const auto inBand = fabs(center) < k_stick_deadband; 

            // Reset controller when moving into deadband above a minimum
            movedIntoBand = inBand && !this->inBandPrev;
            this->errorI = movedIntoBand || reset ? 0 : this->errorI;

            this->inBandPrev = inBand;

            // Target velocity is a setpoint inside deadband, scaled constant
            // outside
            const auto targetVelocity = inBand ?
                target :
                k_pilot_vel_max * center;

            // Compute error as scaled target minus actual
            const auto error = targetVelocity - velocity;

            // Compute I term, avoiding windup
            this->errorI = constrainAbs(this->errorI + error, k_windup_max);

            // Adjust demand based on error
            demand += error * k_p + this->errorI * k_i;
        }

}; // class SetPointPid
