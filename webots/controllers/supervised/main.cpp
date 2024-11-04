/* 
   C++ flight simulator kinematic program 

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

#include <sim/sim2.hpp>
#include <pids/altitude.hpp>
#include <pids/yaw_rate.hpp>
#include <mixers/bfquadx.hpp>

static hf::Simulator _sim;

namespace hf {

    static AltitudePid _altitudePid;

    static YawRatePid _yawRatePid;

    void run_closed_loop_controllers(
            const float dt, const state_t & state, demands_t & demands)
    {
        _altitudePid.run(_sim.isSpringy(), dt, state, demands, demands);

        _yawRatePid.run(dt, false, state, demands);
    }
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::BfQuadXMixer mixer = {};

    _sim.init(mixer);

    while (true) {

        if (!_sim.step()) {
            break;
        }
    }

    _sim.close();

    return 0;
}
