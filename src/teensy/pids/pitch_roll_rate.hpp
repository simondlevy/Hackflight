/*
   Pitch/roll angular rate PID-control algorithm for real and simulated flight
   controllers

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <num.hpp>

/**
  Input is angular rate demands (deg/sec) and actual angular rates from
  gyro; ouputput is arbitrary units scaled for motors
 */
class PitchRollRateController {

    public:

        static void run(
                const bool airborne,
                const float dt, 
                const vehicleState_t & state,
                demands_t & demands)
        {
            static axis_t _roll;

            static axis_t _pitch;

            demands.roll =
                runAxis(dt, airborne, demands.roll, state.dphi, _roll);

            demands.pitch =
                runAxis(dt, airborne, demands.pitch, state.dtheta, _pitch);
        }

    private:

        static constexpr float KP = 0.00025;
        static constexpr float KI = 0.0005;
        static constexpr float KD = 2.5e-6;
        static constexpr float ILIMIT = 33;

        typedef struct {
            float integral;
            float previous;
        } axis_t;

        static float runAxis(
                const bool airborne,
                const float dt,
                const float demand,
                const float measured, 
                axis_t & axis)
        {
            const auto error = demand - measured;

            axis.integral = airborne ? 
                Num::fconstrain(axis.integral + error * dt, ILIMIT) : 0;

            const auto deriv = dt > 0 ? (error - axis.previous) / dt : 0;

            axis.previous = airborne ? error : 0;

            return airborne ?  KP * error + KI * axis.integral + KD * deriv : 0;
        }
};
