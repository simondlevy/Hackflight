/* 
 * Platform-independent support for fast simulator loop (PID control and
 * dynamics)
 *
 * Copyright (C) 2026 Simon D. Levy
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

#include <stdio.h>
#include <time.h>

#include <datatypes.hpp>
#include <num.hpp>
#include <pidcontrol/hover.hpp>
#include <sim/datatypes.hpp>
#include <sim/demixers/bfquadx.hpp>
#include <sim/dynamics.hpp>
#include <sim/vehicles/apexquad.hpp>

namespace hf {

    class Simulator {

        private:

            static constexpr float kDynamicsFreq = 1e5; // Hz
            static constexpr float kPidFastFreq = 500; // 1024 Plank 
            static constexpr float kPidSlowFreq = 100;

        public:

            Dynamics dynamics;

            HoverPidController pid_controller;

            Simulator() = default;

            Simulator(const Pose & pose)
                : dynamics(Dynamics(pose)),
                pid_controller(HoverPidController()) {}

            Simulator(const Dynamics & dynamics,
                    const HoverPidController & pidControl)
                : dynamics(dynamics), pid_controller(pidControl) {}

            static auto Step(
                    const Simulator & sim,
                    const Mode mode,
                    const Setpoint & setpoint,
                    DemixerFun demixer_fun,
                    const float framerate=32) -> Simulator 
            {
                const auto dt = 1/(float)kPidFastFreq;

                auto pidControl = sim.pid_controller;

                auto dynamics = sim.dynamics;

                // Run slow PID control in outer loop -------------------------
                for (uint32_t i=0; i<kPidSlowFreq/framerate; ++i) {

                    // Get vehicle state from dynamics and convert state values
                    // from doubles/radians to floats/degrees for PID
                    // controllers
                    const auto state = SimStateToVehicleState(dynamics.state);

                    // Run fast PID control and mixer in middle loop ----------
                    for (uint32_t j=0; j<kPidFastFreq/kPidSlowFreq; ++j) {

                        // Run PID control to get new setpoint
                        pidControl = HoverPidController::Run(
                                pidControl, dt, mode, state, setpoint);

                        const auto forces = demixer_fun(pidControl.setpoint);

                        // Run dynamics in inner loop -------------------------
                        for (uint32_t k=0; k<kDynamicsFreq/kPidFastFreq; ++k) {
                            dynamics = Dynamics::Update(dynamics,
                                    kVehicleParams, 1 / kDynamicsFreq, forces);
                        }
                    }
                }

                return Simulator(dynamics, pidControl);
            }

        private:

            static auto SimStateToVehicleState(
                    const SimState state) -> VehicleState 
            {
                return VehicleState(
                        (float)state.dx,
                        (float)state.dy,
                        (float)state.z,
                        (float)state.dz,
                        (float)(Num::kRad2Deg * state.phi),
                        (float)(Num::kRad2Deg * state.dphi),
                        (float)(Num::kRad2Deg * state.theta),
                        (float)(Num::kRad2Deg * state.dtheta),
                        (float)(Num::kRad2Deg * state.psi),
                        (float)(Num::kRad2Deg * state.dpsi));
            }
    };
}
