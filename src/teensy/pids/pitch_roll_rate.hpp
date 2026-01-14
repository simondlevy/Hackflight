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

/**
  Input is angular rate demands (deg/sec) and actual angular rates from
  gyro; ouputput is arbitrary units scaled for motors
 */
class PitchRollRatePid {

    private:

        static constexpr float KP = 0.00025;
        static constexpr float KI = 0.0005;
        static constexpr float KD = 2.5e-6;

        static constexpr float ILIMIT = 33;

    public:

        void run(
                const float dt, 
                const bool reset,
                const vehicleState_t & state,
                demands_t & demands,
                const float postScale=1.0)
        {

            runAxis(dt, reset, demands.roll, state.dphi, _roll_integral,
                    _roll_error, postScale);

            //printf("%+3.3f\n", demands.roll);

            runAxis(dt, reset, demands.pitch, state.dtheta, _pitch_integral,
                    _pitch_error, postScale); 
        }

    private:

        float _roll_integral;
        float _roll_error;

        float _pitch_integral;
        float _pitch_error;

        static void runAxis(
                const float dt,
                const bool reset,
                float & demand,
                const float dangle, 
                float & integral,
                float & errprev,
                const float postScale)
        {

            const auto error = demand - dangle;

            integral = reset ? 0 :
                Utils::fconstrain(integral + error * dt, ILIMIT);

            const auto derivative = (error - errprev) / dt;

            demand =
                postScale * (KP * error + KI * integral + KD * derivative);

            errprev = error;
        }
};
