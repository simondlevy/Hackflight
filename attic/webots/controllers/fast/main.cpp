/* 
   C++ flight simulator support for Hackflight Copyright (C) 2024 Simon D. Levy

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

#include <hackflight.hpp>

#include <sim/sim.hpp>

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    while (true) {

        if (!sim.step()) {
            break;
        }

        sim.getDemands();
        sim.getState();

        const float motor = sim.requestedTakeoff() ? 60 : 0;

        const float motors[4] = { motor, motor, motor, motor };

        printf("%f\n", motors[0]);

        sim.setMotors(motors);
    }

    sim.close();

    return 0;
}
