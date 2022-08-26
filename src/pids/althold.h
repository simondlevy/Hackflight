/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "pid.h"

class AltHoldPidController : public PidController {
    
    private:

        static constexpr float ALTITUDE_MIN   = 1.0;
        static constexpr float PILOT_VELZ_MAX = 2.5;
        static constexpr float STICK_DEADBAND = 0.2;
        static constexpr float WINDUP_MAX     = 0.4;

        static bool inBand(float value, float band) 
        {
            return value > -band && value < band;
        }

        static float constrainAbs(float v, float lim)
        {
            return v < -lim ? -lim : v > +lim ? +lim : v;
        }

        float m_kp;
        float m_ki;

        bool m_inBandPrev;
        float m_errorI;
        float m_altitudeTarget;

    public:

        AltHoldPidController(const float kp, const float ki)
        {
            m_kp = kp;
            m_ki = ki;

            m_inBandPrev = false;
            m_errorI = 0;
            m_altitudeTarget = 0;
        }

        virtual void update(
                const uint32_t currentTimeUs,
                Demands * demands,
                const State & vstate,
                const bool reset) override
        {
            (void)currentTimeUs;

            // NED => ENU
            auto altitude = -vstate.z;
            auto dz = -vstate.dz;

            // [0,1] => [-1,+1]
            auto sthrottle = 2 * demands->throttle - 1; 

            // Is stick demand in deadband, above a minimum altitude?
            auto inBand =
                fabs(sthrottle) < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

            // Reset controller when moving into deadband above a minimum altitude
            auto gotNewTarget = inBand && !m_inBandPrev;
            m_errorI = gotNewTarget || reset ? 0 : m_errorI;

            m_inBandPrev = inBand;

            if (reset) {
                m_altitudeTarget = 0;
            }

            m_altitudeTarget = gotNewTarget ? altitude : m_altitudeTarget;

            // Target velocity is a setpoint inside deadband, scaled
            // constant outside
            auto targetVelocity = inBand ?
                m_altitudeTarget - altitude :
                PILOT_VELZ_MAX * sthrottle;

            // Compute error as scaled target minus actual
            auto error = targetVelocity - dz;

            // Compute I term, avoiding windup
            m_errorI = constrainAbs(m_errorI + error, WINDUP_MAX);

            // Adjust throttle demand based on error
            demands->throttle += (error * m_kp + m_errorI * m_ki);
        }

}; // class AltHoldPidController
