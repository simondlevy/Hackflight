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
#include "pids/setpoint.h"

class AltHoldPidController : public PidController {
    
    private:

        SetPointPid pid;

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

            // [0,1] => [-1,+1]
            const auto sthrottle = 2 * demands.throttle - 1; 

            bool movedIntoBand = false;

            pid.modifyDemand(
                k_p,
                k_i,
                k_stick_deadband,
                k_pilot_velz_max,
                k_windup_max,
                vstate.dz,
                this->zTarget - z,
                reset,
                sthrottle,
                demands.throttle,
                movedIntoBand
                );

            if (reset) {
                this->zTarget = 0;
            }

            this->zTarget = movedIntoBand ? z : this->zTarget;
        }

}; // class AltHoldPidController
