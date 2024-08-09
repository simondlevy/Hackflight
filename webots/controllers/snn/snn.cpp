/*
  C++ flight simulator takeoff example for Hackflight
 
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

#include <sim.hpp>
#include <snn.hpp>

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    SNN * snn = NULL;

    try {
        snn = new SNN("networks/takeoff_risp.txt", "risp");
    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    while (true) {

        hf::state_t state = {};

        if (!sim.step(state)) {
            break;
        }

        vector<double> o = {state.z, state.dz};
        vector <double> a;
        snn->getActions(o, a);
        const auto motor = a[0];

        sim.setMotors(motor, motor, motor, motor);
    }

    wb_robot_cleanup();

    return 0;
}
