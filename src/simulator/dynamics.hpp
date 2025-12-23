#pragma once

/*
 * Multirotor flight dynamics for simulators
 *
 * Based on:
 *
 *   @inproceedings{DBLP:conf/icra/BouabdallahMS04,
 *     author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland
 *                  Siegwart},
 *     title     = {Design and Control of an Indoor Micro Quadrotor},
 *     booktitle = {Proceedings of the 2004 {IEEE} International Conference on
 *                  Robotics and Automation, {ICRA} 2004, April 26 - May 1,
 *                  2004, New Orleans, LA, {USA}},
 *     pages     = {4393--4398},
 *     year      = {2004},
 *     crossref  = {DBLP:conf/icra/2004},
 *     url       = {https://doi.org/10.1109/ROBOT.2004.1302409},
 *     doi       = {10.1109/ROBOT.2004.1302409},
 *     timestamp = {Sun, 04 Jun 2017 01:00:00 +0200},
 *     biburl    = {https://dblp.org/rec/bib/conf/icra/BouabdallahMS04},
 *     bibsource = {dblp computer science bibliography, https://dblp.org}
 *   }
 *
 *   Copyright (C) 2025 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, in version 3.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <datatypes.h>

class Dynamics {

    public:

        static constexpr float ZMIN = 0.005;

        // Vehicle state (Equation 11)
        vehicleState_t state;

        /**
         *  Vehicle parameters
         */
        typedef struct {

            // These can be measured directly
            float m;  // mass [kg]
            float l;  // arm length [m]

            // These should be estimated to get realistic behavior
            float b;  // thrust coefficient [F=b*w^2]
            float d;  // drag coefficient [T=d*w^2]
            float I;  // body inertia [kg*m^2]; we ignore x/y/z distinction

        } vehicle_params_t; 

        /**
         *  World parameters
         */
        typedef struct {

            // These can be measured directly
            float g;   // gravitational constant [m/s/s]
            float rho; // air density [kg/m^3]

        } world_params_t; 

        Dynamics(
                const vehicle_params_t & vparams,
                const world_params_t & wparams,
                const float dt)
        {
            init(vparams, wparams, dt);
        }

        Dynamics(const vehicle_params_t & vparams, const float dt)
        {
            const world_params_t wparams = { 9.807, 1.225 };

            init(vparams, wparams, dt);
        }

        /**
         * Sets motor spins
         */
        void update(
                const float * rpms,
                const uint8_t rotorCount,
                const int8_t * roll,
                const int8_t * pitch,
                const int8_t * yaw)
        {
            const auto b = _vparams.b;
            const auto d = _vparams.d;
            const auto I = _vparams.I;
            const auto l = _vparams.l;
            const auto m = _vparams.m;

            // Equation 6 ---------------------------------------

            float u1=0, u2=0, u3=0, u4=0;

            for (unsigned int i = 0; i < rotorCount; ++i) {

                // RPM => rad/sec
                const auto omega = rpms[i] * 2 * M_PI / 60;

                // Thrust is squared rad/sec scaled by air density
                const auto omega2 = _wparams.rho * omega * omega; 

                // Multiply by thrust coefficient
                u1 += b * omega2;                  

                u2 += b * omega2 * roll[i];

                u3 += b * omega2 * pitch[i];

                u4 += d * omega2 * yaw[i];
            }

            // Equation 12 line 6 for dz/dt in inertial (earth) frame
            _dstate.dz =
                -_wparams.g + (cos(state.phi)*cos(state.theta)) / m * u1;

            // We're airborne once net Z acceleration becomes positive
            if (_dstate.dz > 0) {
                _airborne = true;
            }

            // We're no longer airborne if we're descending and drop below
            // minimum altitude
            if (_airborne && state.dz < 0 && state.z < ZMIN) {
                _airborne = false;
                reset();
            }

            // Once airborne, we can update dynamics
            if (_airborne) {

                const auto phi = state.phi;
                const auto theta = state.theta;
                const auto psi = state.psi;

                // Equation 12 : Note negations to support roll-right
                // positive

                _dstate.x = state.dx;

                // Rotate dx/dt from body frame into inertial frame
                _dstate.dx =(cos(-phi)*sin(theta)*cos(psi) +
                        sin(-phi)*sin(psi)) * u1 / m;

                _dstate.y = state.dy;                              

                // Rotate dy/dt from body frame into inertial frame
                _dstate.dy = (cos(-phi)*sin(theta)*sin(psi) -
                        sin(-phi)*cos(psi)) * u1 / m;

                _dstate.z = state.dz;                             
                _dstate.phi = state.dphi;                              
                _dstate.dphi = l / I * u2;                      
                _dstate.theta = state.dtheta;                  
                _dstate.dtheta = l / I * u3;                  
                _dstate.psi = state.dpsi;                   
                _dstate.dpsi = -l / I * u4;                  

                // Compute state as first temporal integral of first
                // temporal derivative
                state.x += _dt * _dstate.x;
                state.dx += _dt * _dstate.dx;
                state.y += _dt * _dstate.y;
                state.dy += _dt * _dstate.dy;
                state.z += _dt * _dstate.z;
                state.dz += _dt * _dstate.dz;
                state.phi += _dt * _dstate.phi;
                state.dphi += _dt * _dstate.dphi;
                state.theta += _dt * _dstate.theta;
                state.dtheta += _dt * _dstate.dtheta;
                state.psi += _dt * _dstate.psi;
                state.dpsi += _dt * _dstate.dpsi;

                // Keep yangle in [-2Pi, +2Pi]
                if (state.psi > 2*M_PI) {
                    state.psi -= 2*M_PI;
                }
                if (state.psi < -2*M_PI) {
                    state.psi += 2*M_PI;
                }
             }

        } // update

        void reset()
        {
            state.dx = 0;
            state.dy = 0;
            state.z = ZMIN;
            state.dz = 0;
            state.dphi = 0;
            state.dtheta = 0;
            state.dpsi = 0;

            memset(&_dstate, 0, sizeof(vehicleState_t));
        }

        // ---------------------------------------------------------------

    private:

        // Vehicle state first derivative (Equation 12)
        vehicleState_t _dstate;

        float _dt;

        vehicle_params_t _vparams;

        world_params_t _wparams;

        // Flag for whether we're airborne and can update dynamics
        bool _airborne = false;

        void init(
                const vehicle_params_t & vparams,
                const world_params_t & wparams,
                const float dt)
        {
            memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

            memcpy(&_wparams, &wparams, sizeof(world_params_t));

            _dt = dt;

            _airborne = false;
        }

}; // class Dynamics
