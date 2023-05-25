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
#include "core/pids/setpoint.h"

class AltHoldPidController : public PidController {
    
    private:

        static bool inBand(const float value, const float band) 
        {
            return value > -band && value < band;
        }

        static float constrainAbs(const float v, const float lim)
        {
            return v < -lim ? -lim : v > +lim ? +lim : v;
        }

        float k_p;
        float k_i;

        float k_alt_min;
        float k_pilot_velz_max;
        float k_stick_deadband;
        float k_windup_max;

        float zTarget;

        bool inBandPrev;
        float errorI;

    public:

        AltHoldPidController(

                // Tunable
                const float k_p = 0.075,
                const float k_i = 0.15,

                // Probably better left as-is
                const float k_alt_min = 1.0,
                const float k_pilot_velz_max = 2.5,
                const float k_stick_deadband = 0.2,
                const float k_windup_max = 0.4)
        {
            this->k_p = k_p;
            this->k_i = k_i;

            this->k_alt_min = k_alt_min;
            this->k_pilot_velz_max = k_pilot_velz_max;
            this->k_stick_deadband = k_stick_deadband;
            this->k_windup_max = k_windup_max;

            this->inBandPrev = false;
            this->errorI = 0;
            this->zTarget = 0;
        }

        virtual void modifyDemands(
                Demands & demands,
                const int32_t dusec,
                const VehicleState & vstate,
                const bool reset) override
         {
            (void)dusec;

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
            const auto movedIntoBand = inBand && !this->inBandPrev;
            this->errorI = movedIntoBand || reset ? 0 : this->errorI;

            this->inBandPrev = inBand;

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

            if (reset) {
                this->zTarget = 0;
            }

            this->zTarget = movedIntoBand ? z : this->zTarget;
        }

}; // class AltHoldPidController
