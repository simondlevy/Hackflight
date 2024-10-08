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
                    printf("%f\n", _z_target);
                }
            }

        private:

            static constexpr float DEADBAND = 0.2;
    };

}
