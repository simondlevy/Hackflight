/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "../closedloop.hpp"
#include "../math3d.h"
#include "../pid.hpp"

class PositionController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float altitudeMin,
                const float kp=25, 
                const float ki=1)
        {
            ClosedLoopController::init(updateRate);

            _altitudeMin = altitudeMin;

            initAxis(_pidX, kp, ki);
            initAxis(_pidY, kp, ki);
        }

        /**
          * Demands are input as normalized interval [-1,+1] and output as
          * angles in degrees.
          *
          * roll:  input right negative => output negative
          *
          * pitch: input forward positive => output negative
          */
         virtual void run(const vehicleState_t & state,
                demands_t & demands) override 
        {
            // Rotate world-coordinate velocities into body coordinates
            const auto dxw = state.dx;
            const auto dyw = state.dy;
            const auto psi = deg2rad(state.psi);
            const auto cospsi = cos(psi);
            const auto sinpsi = sin(psi);
            const auto dxb =  dxw * cospsi + dyw * sinpsi;
            const auto dyb = -dxw * sinpsi + dyw * cospsi;       

            // Run PID closedloops on body-coordinate velocities
            demands.roll  = runAxis(state, demands.roll, dyb, _pidY);
            demands.pitch = runAxis(state, demands.pitch, dxb, _pidX);
        }

        void resetPids(void)
        {
            _pidX.reset();
            _pidY.reset();
        }

        void resetFilters(void)
        {
            resetFilter(_pidX);
            resetFilter(_pidY);
        }

    private:

        static constexpr float LIMIT = 20;
        static constexpr float LIMIT_OVERHEAD = 1.10;
        static constexpr float FILTER_CUTOFF = 20;

        static float deg2rad(float degrees) 
        { 
            return (M_PI / 180.0f) * degrees; 
        }

        Pid _pidX;
        Pid _pidY;

        float _altitudeMin;

        void resetFilter(Pid & pid)
        {
            pid.filterReset(_updateRate, FILTER_CUTOFF, true);
        }

        void initAxis(Pid & pid, const float kp, const float ki)
        {
            pid.init(kp, ki, 0, 0, _dt, _updateRate, FILTER_CUTOFF, true);
            pid.setOutputLimit(LIMIT * LIMIT_OVERHEAD);
        }

        float runAxis(
                const vehicleState_t & state, 
                const float demand, 
                const float measured, Pid & pid)
        {
            // Position data is unreliable below a certain altitude, below
            // which we return a zero demand.  To accommodate the demand and
            // state conventions (+Y left / roll left negative; +X forward /
            // nose-up positive), we negate the output of the PID controller.
            return state. z > _altitudeMin ? 
                Num::fconstrain(-pid.run(demand, measured), -LIMIT, +LIMIT) :
                0;
        }

};
