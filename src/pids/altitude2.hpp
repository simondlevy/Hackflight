/*
   Altitude PID controller, version 2: update altitude target inside deabdand;
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
            }
    };

}
