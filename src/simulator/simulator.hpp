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
#include <pidcontrol.hpp>
#include <simulator/dynamics.hpp>
#include <vehicles/diyquad.hpp>

class Simulator {

    private:

        static constexpr float DYNAMICS_FREQ = 1e5; // Hz
        static constexpr float PID_FAST_FREQ = 500; // 1024 Plank 
        static constexpr float PID_SLOW_FREQ = 100;

    public:

        void init(PidControl * pidControl)
        {
            _pidControl = pidControl;

            _pidControl->init();
        }

        void setPoseAndFramerate(
                const Dynamics::pose_t & pose, const float framerate)
        {
            _dynamics.setPose(pose);

            _framerate= framerate;
        }

        Dynamics::pose_t step(const mode_e mode, const demands_t & setpoint)
        {
            // Run slow PID control in outer loop ----------------------------
            for (uint32_t i=0; i<PID_SLOW_FREQ/_framerate; ++i) {

                // Get vehicle state from dynamics and convert state values
                // from doubles to floats
                const auto state = state2floats(
                        _dynamics.getVehicleStateDegrees());

                const bool controlled =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const demands_t sp = setpoint;

                demands_t setpoint = { sp.thrust, sp.roll, sp.pitch, sp.yaw };

                const float dt = 1/(float)PID_FAST_FREQ;

                // Run fast PID control and mixer in middle loop --------------
                for (uint32_t j=0; j<PID_FAST_FREQ/PID_SLOW_FREQ; ++j) {

                    demands_t demands = {};

                    _pidControl->run(dt, controlled, state, setpoint, demands);

                    // Get motor RPMS from mixer
                    float motors[4] = {};
                    Mixer::mix(demands, motors);

                    // Convert motor values to double for dynamics
                    double rpms[4] = {};
                    floats2doubles(motors, rpms, 4);

                    // Run dynamics in inner loop -----------------------------
                    for (uint32_t k=0; k<DYNAMICS_FREQ/PID_FAST_FREQ; ++k) {

                        _dynamics.update(rpms, 4,
                                Mixer::roll, Mixer::pitch, Mixer::yaw);
                    }
                }
            }

            return _dynamics.getPose();
        }    

    private:

        Dynamics _dynamics = Dynamics(VPARAMS, 1./DYNAMICS_FREQ);

        PidControl * _pidControl;

        float _framerate;

        static vehicleState_t state2floats(const Dynamics::state_t s)
        {
            return vehicleState_t { 
                (float)s.x, (float)s.dx, (float)s.y, (float)s.dy,
                (float)s.z, (float)s.dz, (float)s.phi, (float)s.dphi,
                (float)s.theta, (float)s.dtheta, (float)s.psi,
                (float)s.dpsi
            };
        }

        void floats2doubles(const float * f, double * d, const size_t n)
        {
            for (size_t k=0; k<n; ++k) {
                d[k] = f[k];
            }
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

