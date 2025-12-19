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

#include <datatypes.h>
#include <mixers/crazyflie.hpp>
#include <num.hpp>
#include <pid.hpp>
#include <simulator/dynamics.hpp>
#include <vehicles/crazyflie.hpp>

class Simulator {

    private:

        static constexpr float DYNAMICS_FREQ = 1e5; // Hz

    public:

        typedef struct {

            float x;
            float y;
            float z;
            float phi;
            float theta;
            float psi;

        } pose_t;


        typedef struct {

            float start_x;
            float start_y;
            float start_z;
            float framerate;
            char path[200];
            char worldname[200];
            flightMode_t flightMode;
            demands_t setpoint;

        } info_t;

        void init(PidControl * pidControl)
        {
            _pidControl = pidControl;

            _pidControl->init();
        }

        pose_t step(const info_t & siminfo)
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

        Dynamics _dynamics = Dynamics(VPARAMS, 1./DYNAMICS_FREQ);

        PidControl * _pidControl;

        vehicleState_t getVehicleState()
        {
            const auto d = _dynamics;

            return vehicleState_t {
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

