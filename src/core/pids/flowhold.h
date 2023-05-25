/*
   Copyright (c) 2023 Simon D. Levy

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

class FlowHoldPidController : public PidController {
    
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

        float k_pilot_vely_max;
        float k_stick_deadband;
        float k_windup_max;

        bool m_inBandPrev;
        float m_errorI;

    public:

        FlowHoldPidController(

                // Tunable
                const float k_p = 0.0005,
                const float k_i = 0.25,

                // Probably better left as-is
                const float k_pilot_vely_max = 2.5,
                const float k_stick_deadband = 0.2,
                const float k_windup_max = 0.4)
        {
            this->k_p = k_p;
            this->k_i = k_i;

            this->k_pilot_vely_max = k_pilot_vely_max;
            this->k_stick_deadband = k_stick_deadband;
            this->k_windup_max = k_windup_max;

            m_inBandPrev = false;
            m_errorI = 0;
        }

        virtual void modifyDemands(
                Demands & demands,
                const int32_t dusec,
                const VehicleState & vstate,
                const bool reset) override
         {
            (void)dusec;

            const auto dy = vstate.dy;
            const auto roll = demands.roll; 

            // Is stick demand in deadband?
            const auto inBand = fabs(roll) < k_stick_deadband; 

            // Reset controller when moving into deadband above a minimum
            // altitude
            const auto gotNewTarget = inBand && !m_inBandPrev;
            m_errorI = gotNewTarget || reset ? 0 : m_errorI;

            m_inBandPrev = inBand;

            // Compute error as scaled target minus actual
            const auto error = 0 - dy;

            // Compute I term, avoiding windup
            m_errorI = constrainAbs(m_errorI + error, k_windup_max);

            // Adjust throttle demand based on error
            demands.roll += error * k_p + m_errorI * k_i;
        }

}; // class FlowHoldPidController
