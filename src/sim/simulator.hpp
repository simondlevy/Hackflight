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

#include <datatypes.h>
#include <mixers/crazyflie.hpp>
#include <num.hpp>
#include <new_pidcontrol.hpp>
#include <sim/dynamics.hpp>
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

            void init(const Dynamics::pose_t & pose, const float framerate=32)
            {
                _pidControl.init();

                _dynamics.reset();

                _dynamics.state.x = pose.x;
                _dynamics.state.y = pose.y;
                _dynamics.state.z = pose.z;
                _dynamics.state.phi = pose.phi;
                _dynamics.state.theta = pose.theta;
                _dynamics.state.psi = pose.psi;

                _framerate= framerate;
            }

            Dynamics::State step(const mode_e mode, const setpoint_t & setpoint)
            {
                const auto controlled =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const auto dt = 1/(float)PID_FAST_FREQ;

                // Run slow PID control in outer loop ----------------------------
                for (uint32_t i=0; i<PID_SLOW_FREQ/_framerate; ++i) {

                    // Get vehicle state from dynamics and convert state values
                    // from doubles/radians to floats/degrees for PID
                    // controllers
                    const auto state = state2degrees(_dynamics.state);

                    // Run fast PID control and mixer in middle loop --------------
                    for (uint32_t j=0; j<PID_FAST_FREQ/PID_SLOW_FREQ; ++j) {

                        // Run PID control to get new setpoint
                        const auto pid_setpoint =
                            _pidControl.run(dt, controlled, state, setpoint);

                        // Scale up new setpoint for mixer
                        const setpoint_t scaled_setpoint = {
                            pid_setpoint.thrust,
                            pid_setpoint.roll * PITCH_ROLL_MOTOR_SCALE,
                            pid_setpoint.pitch * PITCH_ROLL_MOTOR_SCALE,
                            pid_setpoint.yaw * YAW_MOTOR_SCALE
                        };

                        // Get motor RPMS from mixer
                        const auto motors = Mixer::mix(scaled_setpoint);

                        // Convert motor values to double for dynamics
                        const auto rpms = motors2doubless(motors, 4);

                        // Run dynamics in inner loop -----------------------------
                        for (uint32_t k=0; k<DYNAMICS_FREQ/PID_FAST_FREQ; ++k) {

                            _dynamics.update( VPARAMS, 1 / DYNAMICS_FREQ, rpms,
                                    4, Mixer::roll, Mixer::pitch, Mixer::yaw);
                        }
                    }
                }

                return _dynamics.state;
            }

            Dynamics::State getVehicleState()
            {
                return _dynamics.state;
            }

        private:

            Dynamics _dynamics;

            PidControl _pidControl;

            float _framerate;

            static vehicleState_t state2floats(const Dynamics::State s)
            {
                return vehicleState_t { 
                    (float)s.x, (float)s.dx, (float)s.y, (float)s.dy,
                        (float)s.z, (float)s.dz, (float)s.phi, (float)s.dphi,
                        (float)s.theta, (float)s.dtheta, (float)s.psi,
                        (float)s.dpsi
                };
            }

            static double * motors2doubless(const float * f, const size_t n)
            {
                static double d[MAX_MOTOR_COUNT];
                for (size_t k=0; k<n; ++k) {
                    d[k] = f[k];
                }
                return d;
            }

            static vehicleState_t state2degrees(const Dynamics::State state)
            {
                return vehicleState_t {
                    (float)state.x,
                        (float)state.dx,
                        (float)state.y,
                        (float)state.dy,
                        (float)state.z,
                        (float)state.dz,
                        (float)(Num::RAD2DEG * state.phi),
                        (float)(Num::RAD2DEG * state.dphi),
                        (float)(Num::RAD2DEG * state.theta),
                        (float)(Num::RAD2DEG * state.dtheta),
                        (float)(Num::RAD2DEG * state.psi),
                        (float)(Num::RAD2DEG * state.dpsi)
                };
            }


            static void report_fps()
            {
                static uint32_t _count;
                static uint32_t _sec_prev;

                time_t now = time(0);
                struct tm * tm = localtime(&now);
                const uint32_t sec_curr = tm->tm_sec;
                if (sec_curr - _sec_prev >= 1) {
                    if (_sec_prev > 0) {
                        printf("%d\n", _count);
                    }
                    _sec_prev = sec_curr;
                    _count = 0;
                }
                _count++;
            }
    };
}
