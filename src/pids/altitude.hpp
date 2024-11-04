/*
   Altitude PID controller

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

#include <utils.hpp>

namespace hf {

    class AltitudePid {

        public:

            void run(
                    const bool throttleIsSpringy,
                    const float dt,
                    const state_t & state,
                    demands_t & demands)
            {
                // "Springy" (self-centering) throttle or keyboard: accumulate 
                // altitude target based on stick deflection, and attempt
                // to maintain target via PID control
                if (throttleIsSpringy) {

                    _z_target += dt * demands.thrust;
                    demands.thrust = _z_target;
                    _run(dt, state, demands);
                }

                // Traditional (non-self-centering) throttle: 
                //
                //   (1) In throttle deadband (mid position), fix an altitude target
                //       and attempt to maintain it via PID control
                //
                //   (2) Outside throttle deadband, get thrust from stick deflection
                else {

                    static bool _was_in_deadband;
                    const auto in_deadband = fabs(demands.thrust) < THROTTLE_DEADBAND;
                    _z_target = in_deadband && !_was_in_deadband ? state.z : _z_target;

                    _was_in_deadband = in_deadband;

                    if (in_deadband) {
                        demands.thrust = _z_target;
                        _run(dt, state, demands);
                    }
                    else {
                        demands.thrust = demands.thrust;
                    }
                }
            }

        private:

            static constexpr float KP_Z = 2.0;
            static constexpr float KI_Z = 0.5;

            static constexpr float KP_DZ = 25;
            static constexpr float KI_DZ = 15;

            static constexpr float ILIMIT = 5000;

            // For springy-throttle in gamepads / keyboard
            static constexpr float INITIAL_ALTITUDE_TARGET = 0.2;

            // For tradtional (non-springy) throttle in R/C transmitter
            static constexpr float THROTTLE_DEADBAND = 0.2;

            float _z_target = INITIAL_ALTITUDE_TARGET;

            float _z_integral;
            float _dz_integral;

            void _run(
                    const float dt, const state_t & state, demands_t & demands)
            {
                demands.thrust = _run_pi(dt, KP_Z, KI_Z,
                        demands.thrust, state.z, _z_integral);

                demands.thrust = _run_pi(dt, KP_DZ, KI_DZ,
                        demands.thrust, state.dz, _dz_integral);
            }

            static float _run_pi(const float dt, const float kp, const float ki,
                    const float target, const float actual, float & integral)
            {
                const auto error = target - actual;

                integral = Utils::fconstrain(integral + dt * error, ILIMIT);

                return kp * error + ki * integral;
            }
    };
}
