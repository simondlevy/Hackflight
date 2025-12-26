/* 
   C++ flight simulator outer loop for Hackflight

   Copyright (C) 2025 Simon D. Levy

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


#include <simulator/outer.hpp>

// Simple version is always in manual mode
static bool is_flight_mode_autonomous(const flightMode_t mode)
{
    (void)mode;
    return false;
}

int main() 
{
    SimOuterLoop outerLoop = {};

    outerLoop.begin();

    while (true) {

        siminfo_t siminfo = {};

        if (!outerLoop.beginStep(is_flight_mode_autonomous, siminfo)) {
            break;
        }

        outerLoop.endStep(siminfo);
    }

    return outerLoop.end();
}
