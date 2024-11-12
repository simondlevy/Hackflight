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
#include <sim.hpp>

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    auto logfp = fopen("log.csv", "w");

    float motors[4] = {};

    while (true) {

        if (!sim.step()) {
            break;
        }

        sim.getDemands();

        const auto state = sim.getState();

        const float MOTOR = 55.385; // rad/sec

        motors[0] = MOTOR;
        motors[1] = MOTOR;
        motors[2] = MOTOR;
        motors[3] = MOTOR;

        fprintf(logfp, "%f,%f,%f\n", sim.getTime(), motors[0], state.z);

        sim.setMotors(motors);
    }

    sim.close();

    return 0;
}
