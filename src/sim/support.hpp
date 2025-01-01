/* 
 * Simulation support for Hackflight
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.hpp>
#include <mixers/bfquadx.hpp>
#include <sim/dynamics.hpp>
#include <sim/vehicles/diyquad.hpp>

static const float DYNAMICS_RATE = 100000; // Hz

static const float PID_RATE = 1000; // Hz

namespace hf {

    static Dynamics _dynamics = hf::Dynamics(hf::VPARAMS, 1./DYNAMICS_RATE);

    // Write these for each paradigm (standard, snn, haskell, ekf) ------------

    void setup_controllers();

    void setup_estimator();

    state_t estimate_state(
            const Dynamics & dynamics, const float pid_rate);

    demands_t run_controllers(
            const float pid_dt,
            const siminfo_t & siminfo,
            const state_t & state);

    // -----------------------------------------------------------------------

    static pose_t run_sim_middle_loop(const siminfo_t & siminfo)
    {
        bool landed = false;

        // Run control in middle loop
        for (uint32_t j=0;
                j < (uint32_t)(1 / siminfo.framerate * PID_RATE);  ++j) {

            const auto state = estimate_state(_dynamics, PID_RATE);

            const auto demands = run_controllers(1 / PID_RATE, siminfo, state);

            BfQuadXMixer mixer = {};

            float motors[4] = {};

            mixer.run(demands, motors);

            // Run dynamics in innermost loop
            for (uint32_t k=0; k<DYNAMICS_RATE / PID_RATE; ++k) {

                _dynamics.update(motors, &mixer);

                if (_dynamics.state.z < 0) {
                    landed = true;
                    break;
                }
            }
        }

        if (landed) {
            printf("landed\n");
        }

        // Get current pose from dynamics
        return _dynamics.getPose();
    }
}
