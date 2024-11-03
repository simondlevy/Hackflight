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

        auto demands = sim.getDemands();

        const auto state = sim.getState();

        (void)demands;
        (void)state;

        /*


           demands.yaw *= YAW_PRESCALE;

           const auto resetPids = demands.thrust < THROTTLE_DOWN;

        // Throttle control begins when once takeoff is requested, either by
        // hitting a button or key ("springy", self-centering throttle) or by
        // raising the non-self-centering throttle stick
        if (sim.requestedTakeoff()) {

        // "Springy" (self-centering) throttle or keyboard: accumulate 
        // altitude target based on stick deflection, and attempt
        // to maintain target via PID control
        if (sim.isSpringy()) {

        z_target += CLIMB_RATE_SCALE * demands.thrust;
        demands.thrust = z_target;
        altitudePid.run(DT, state, demands);
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
        z_target = in_deadband && !_was_in_deadband ? state.z : z_target;
        _was_in_deadband = in_deadband;
        if (in_deadband) {
        demands.thrust = z_target;
        altitudePid.run(DT, state, demands);
        }
        }

        demands.thrust += THRUST_BASE;
        }    
         */
    }

    sim.close();

    return 0;
}
