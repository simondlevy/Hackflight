/* 
 * Platform-independent support for simulator inner loop (PID control /
 * dynamics)
 *
 * Copyright (C) 2025 Simon D. Levy
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
#include <pid.hpp>
#include <simulator/dynamics.hpp>
#include <simulator/common.h>
#include <vehicles/crazyflie.hpp>

class SimInnerLoop {

    private:

        static constexpr float DYNAMICS_FREQ = 1e5; // Hz

    public:

        void init(PidControl * pidControl)
        {
            _pidControl = pidControl;

            _pidControl->init();
        }

        pose_t step(const siminfo_t & siminfo)
        {
            // Run slow PID control in outer loop ----------------------------
            for (uint32_t i=0; i<PID_SLOW_FREQ/siminfo.framerate; ++i) {

                const auto state =  getVehicleState();
                demands_t slowDemands = {};

                const bool controlled = siminfo.flightMode == MODE_HOVERING ||
                    siminfo.flightMode == MODE_AUTONOMOUS;

                _pidControl->runSlow(1 / (float)PID_SLOW_FREQ,
                        controlled, state, siminfo.setpoint, slowDemands);

                // Run fast PID control and mixer in middle loop --------------
                for (uint32_t j=0; j<PID_FAST_FREQ/PID_SLOW_FREQ; ++j) {

                    demands_t fastDemands = {};

                    _pidControl->runFast(1 / (float)PID_FAST_FREQ,
                            controlled, state, slowDemands, fastDemands);

                    float motors[4] = {};
                    Mixer::mix(fastDemands, motors);

                    // Run dynamics in inner loop ----------------------------
                    for (uint32_t k=0; k<DYNAMICS_FREQ/PID_FAST_FREQ; ++k) {

                        _dynamics.update(motors, Mixer::rotorCount,
                                Mixer::roll, Mixer::pitch, Mixer::yaw);
                    }
                }
            }

            const auto s = _dynamics.state;
            const auto p = siminfo.startingPose;

            return pose_t {
                s.x + p.x, s.y + p.y, s.z + p.z,
                s.phi, s.theta, s.psi
            };
        }    

    private:

        Dynamics _dynamics = Dynamics(VPARAMS, 1./DYNAMICS_FREQ);

        PidControl * _pidControl;

        vehicleState_t getVehicleState()
        {
            const auto s = _dynamics.state;

            return vehicleState_t {
                s.x, s.dx, s.y, s.dy, s.z, s.dz,                   
                    r2d(s.phi), r2d(s.dphi), 
                    r2d(s.theta), r2d(s.dtheta),
                    r2d(s.psi),  r2d(s.dpsi)
            };

        }

        static float r2d(const float r)
        {
            return Num::RAD2DEG * r;
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

