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
#include <mixers/bfquadx.hpp>
#include <num.hpp>
#include <pidcontrol/new_pidcontrol.hpp>
#include <sim/dynamics.hpp>
#include <sim/state.hpp>
#include <vehicles/diyquad.hpp>

namespace hf {

    class Simulator {

        private:

            static constexpr float DYNAMICS_FREQ = 1e5; // Hz
            static constexpr float PID_FAST_FREQ = 500; // 1024 Plank 
            static constexpr float PID_SLOW_FREQ = 100;

            static constexpr float PITCH_ROLL_MOTOR_SCALE = 1e6;
            static constexpr float YAW_MOTOR_SCALE = 2e4;

        public:

            Dynamics dynamics;

            Simulator() = default;

            Simulator(const pose_t & pose)
                : dynamics(Dynamics(pose)), _pidControl(PidControl()) {}

            Simulator(const Dynamics & dynamics, const PidControl & pidControl)
                : dynamics(dynamics), _pidControl(pidControl) {}

            static auto step(
                    const Simulator & sim,
                    const mode_e mode,
                    const Setpoint & setpoint,
                    const float framerate=32) -> Simulator 
            {
                const auto controlled =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const auto dt = 1/(float)PID_FAST_FREQ;

                auto pidControl = sim._pidControl;

                auto dynamics = sim.dynamics;

                // Run slow PID control in outer loop ----------------------------
                for (uint32_t i=0; i<PID_SLOW_FREQ/framerate; ++i) {

                    // Get vehicle state from dynamics and convert state values
                    // from doubles/radians to floats/degrees for PID
                    // controllers
                    const auto state = state2degrees(dynamics.state);

                    // Run fast PID control and mixer in middle loop --------------
                    for (uint32_t j=0; j<PID_FAST_FREQ/PID_SLOW_FREQ; ++j) {

                        // Run PID control to get new setpoint
                        // PidControl::run(_pidControl, dt, controlled, state, setpoint);
                        pidControl = PidControl::run(
                                pidControl, dt, controlled, state, setpoint);

                        // Scale up new setpoint for mixer
                        const Setpoint scaled_setpoint = {
                            pidControl.setpoint.thrust,
                            pidControl.setpoint.roll * PITCH_ROLL_MOTOR_SCALE,
                            pidControl.setpoint.pitch * PITCH_ROLL_MOTOR_SCALE,
                            pidControl.setpoint.yaw * YAW_MOTOR_SCALE
                        };

                        // Get motor RPMS from mixer
                        static hf::Mixer _mixer;
                        _mixer = hf::Mixer::run(_mixer, scaled_setpoint);

                        // Convert motor values to double for dynamics
                        const auto rpms = motors2doubless(_mixer.motorvals, 4);

                        // Run dynamics in inner loop -----------------------------
                        for (uint32_t k=0; k<DYNAMICS_FREQ/PID_FAST_FREQ; ++k) {

                            dynamics = Dynamics::update(dynamics,
                                    VPARAMS, 1 / DYNAMICS_FREQ, rpms, 4,
                                    _mixer.roll, _mixer.pitch, _mixer.yaw); }
                    }
                }

                return Simulator(dynamics, pidControl);
            }

        private:

            PidControl _pidControl;

            static double * motors2doubless(const float * f, const size_t n)
            {
                static double d[MAX_MOTOR_COUNT];
                for (size_t k=0; k<n; ++k) {
                    d[k] = f[k];
                }
                return d;
            }

            static VehicleState state2degrees(const State state)
            {
                return VehicleState(
                        (float)state.dx,
                        (float)state.dy,
                        (float)state.z,
                        (float)state.dz,
                        (float)(Num::RAD2DEG * state.phi),
                        (float)(Num::RAD2DEG * state.dphi),
                        (float)(Num::RAD2DEG * state.theta),
                        (float)(Num::RAD2DEG * state.dtheta),
                        (float)(Num::RAD2DEG * state.psi),
                        (float)(Num::RAD2DEG * state.dpsi));
            }
    };
}
