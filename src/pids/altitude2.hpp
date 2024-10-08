/*
   Altitude PID controller, version 2: update altitude target inside deadbdand;
   outside deadband, move proportional to stick throw.

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

namespace hf {

    class AltitudePid2 {

        public:

            void run(
                    const float dt, const state_t & state, demands_t & demands)
            {
                static float _z_target;
                static bool _was_in_deadband;

                const auto in_deadband = fabs(demands.thrust) < DEADBAND;

                _z_target = in_deadband && !_was_in_deadband ? state.z : _z_target;

                _was_in_deadband = in_deadband;

                if (in_deadband) {

                    demands.thrust = _z_target;

                    demands.thrust = run_pi(dt, KP_Z, KI_Z,
                            demands.thrust, state.z, _z_integral);

                    demands.thrust = run_pi(dt, KP_DZ, KI_DZ,
                            demands.thrust, state.dz, _dz_integral);
                }
            }

        private:

            static constexpr float KP_Z = 2.0;
            static constexpr float KI_Z = 0.5;

            static constexpr float KP_DZ = 25;
            static constexpr float KI_DZ = 15;

            static constexpr float ILIMIT = 5000;

            static constexpr float DEADBAND = 0.2;

            float _z_integral;
            float _dz_integral;

            static float run_pi(const float dt, const float kp, const float ki,
                    const float target, const float actual, float & integral)
            {
                const auto error = target - actual;

                integral = Utils::fconstrain(integral + dt * error, ILIMIT);

                return  kp * error + ki * integral;
            }
    };
}
