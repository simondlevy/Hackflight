/* 
 * Hackflight simulator support
 *
 *  Copyright (C) 2025 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
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

#include <pid.hpp>
#include <datatypes.h>
#include <num.hpp>
#include <mixers/crazyflie.hpp>
#include <simulator/dynamics.hpp>
#include <vehicles/diyquad.hpp>

class Simulator {

    private:

        static constexpr float DYNAMICS_RATE = 100000; // Hz

        static const int PID_UPDATE_RATE = 1000;  // 1024 Plank 

    public:

        typedef struct {

            float x;
            float y;
            float z;
            float phi;
            float theta;
            float psi;

        } pose_t;

        void init(PidControl * closedLoopControl)
        {

            _closedLoopControl = closedLoopControl;

            _closedLoopControl->init();
        }

        pose_t step(const siminfo_t & siminfo)
        {

            // Run control in outer loop
            for (uint32_t j=0; j<PID_UPDATE_RATE/siminfo.framerate; ++j) {

                float motors[4] = {};
                outerLoop(siminfo, motors);

                // Run dynamics in inner loop
                for (uint32_t k=0; k<DYNAMICS_RATE/PID_UPDATE_RATE; ++k) {

                    _dynamics.update(motors, Mixer::rotorCount,
                            Mixer::roll, Mixer::pitch, Mixer::yaw);
                }
            }

            // Get current pose from dynamics
            const auto s = _dynamics.state;
            return pose_t { s.x, s.y, s.z, s.phi, s.theta, s.psi };
        }    

    private:

        // For debugging
        const char * modeNames[6] = {
            "IDLE",
            "ARMED",
            "HOVERING",
            "AUTONOMOUS",
            "LANDING",
            "LOST_CONTACT"
        };

        Dynamics _dynamics = Dynamics(VPARAMS, 1./DYNAMICS_RATE);

        PidControl * _closedLoopControl;

        void outerLoop(const siminfo_t & siminfo, float * motors)
        {
            const auto d = _dynamics;

            const vehicleState_t state =  {
                0, // x
                d.state.dx,
                0, // y
                d.state.dy,
                d.state.z,                     
                d.state.dz,                   
                Num::RAD2DEG* d.state.phi, 
                Num::RAD2DEG* d.state.dphi, 
                Num::RAD2DEG* d.state.theta, 
                Num::RAD2DEG* d.state.dtheta,
                Num::RAD2DEG* d.state.psi,   
                Num::RAD2DEG* d.state.dpsi
            };

            demands_t demands = {};

            if (siminfo.flightMode != MODE_IDLE) {

                _closedLoopControl->run(
                        1 / (float)PID_UPDATE_RATE,
                        siminfo.flightMode,
                        state,
                        siminfo.setpoint,
                        demands);

                demands.roll *= Num::DEG2RAD;
                demands.pitch *= Num::DEG2RAD;
                demands.yaw *= Num::DEG2RAD;

                Mixer::mix(demands, motors);

                if (_dynamics.state.z < 0) {
                    _dynamics.reset();
                }
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

