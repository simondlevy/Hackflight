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

#include "core/pid.h"

class SetPointPidController {
    
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

        SetPointPidController(void)
        {
            this->inBandPrev = false;
            this->errorI = 0;
        }

        virtual void modifyDemand(
                const VehicleState & vstate,
                const bool reset,
                float & demand,
                bool & movedIntoBand
                ) 
         {
            const auto z = vstate.z;

            // Require a minimum altitude
            if (z < k_alt_min) {
                return;
            }

            const auto dz = vstate.dz;

            // [0,1] => [-1,+1]
            const auto sthrottle = 2 * demands.throttle - 1; 

            // Is stick demand in deadband?
            const auto inBand = fabs(sthrottle) < k_stick_deadband; 

            // Reset controller when moving into deadband above a minimum
            // z
            movedIntoBand = inBand && !this->inBandPrev;
            this->errorI = movedIntoBand || reset ? 0 : this->errorI;

            this->inBandPrev = inBand;

            if (reset) {
                this->zTarget = 0;
            }

            this->zTarget = movedIntoBand ? z : this->zTarget;

            // Target velocity is a setpoint inside deadband, scaled constant
            // outside
            const auto targetVelocity = inBand ?
                this->zTarget - z :
                k_pilot_velz_max * sthrottle;

            // Compute error as scaled target minus actual
            const auto error = targetVelocity - dz;

            // Compute I term, avoiding windup
            this->errorI = constrainAbs(this->errorI + error, k_windup_max);

            // Adjust throttle demand based on error
            demands.throttle += error * k_p + this->errorI * k_i;
        }

}; // class SetPointPidController
