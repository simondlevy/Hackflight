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

                const auto state = _dynamics.getVehicleStateDegrees();

                const bool controlled =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const demands_t sp = setpoint;

                demands_t setpoint = { sp.thrust, sp.roll, sp.pitch, sp.yaw };

                // Run fast PID control and mixer in middle loop --------------
                for (uint32_t j=0; j<PID_FAST_FREQ/PID_SLOW_FREQ; ++j) {

                    demands_t demands = {};

                    _pidControl->run(
                            1 / (float)PID_FAST_FREQ,
                            controlled,
                            {
                            state.x, state.dx,
                            state.y, state.dy,
                            state.z, state.dz,
                            state.phi, state.dphi,
                            state.theta, state.dtheta,
                            state.psi, state.dpsi
                            },
                            setpoint,
                            demands);

                    float motors[4] = {};
                    Mixer::mix(demands, motors);

                    // Run dynamics in simulator loop ----------------------------
                    for (uint32_t k=0; k<DYNAMICS_FREQ/PID_FAST_FREQ; ++k) {

                        _dynamics.update(motors, Mixer::rotorCount,
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

