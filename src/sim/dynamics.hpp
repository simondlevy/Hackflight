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
 *   Copyright (C) 2024 Simon D. Levy
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

#pragma once

#include <string.h>

#include <mixer.hpp>
#include <utils.hpp>

namespace hf {

    class Dynamics {

        friend class GroundTruth;
        friend class Gyrometer;
        friend class Accelerometer;
        friend class OpticalFlow;
        friend class Rangefinder;

        public:

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
             *
             * @param omegas motor spins in radians per second
             * @param mixer, used for rotor directions
             */
            void update(const float * omegas, Mixer * mixer) 
            {
                const auto b = _vparams.b;
                const auto d = _vparams.d;
                const auto I = _vparams.I;
                const auto l = _vparams.l;
                const auto m = _vparams.m;

                // Equation 6 ---------------------------------------

                float u1=0, u2=0, u3=0, u4=0;

                for (unsigned int i = 0; i < mixer->rotorCount(); ++i) {

                    // Thrust is squared rad/sec scaled by air density
                    const auto omega2 = _wparams.rho * omegas[i] * omegas[i]; 

                    // Multiply by thrust coefficient
                    u1 += b * omega2;                  

                    u2 += b * omega2 * mixer->roll(i);

                    u3 += b * omega2 * mixer->pitch(i);

                    u4 += d * omega2 * mixer->yaw(i);
                }

                // Equation 12 line 6 for dz/dt in inertial (earth) frame
                _dstate.dz = -_wparams.g + (cos(x7)*cos(x9)) * 1 / m * u1;

                // We're airborne once net Z acceleration becomes positive
                if (_dstate.dz > 0) {
                    _airborne = true;
                }

                // Once airborne, we can update dynamics
                if (_airborne) {

                    // Equation 12 : Note negations to support roll-right
                    // positive

                    _dstate.x = x2;                               // x
                    _dstate.dx =(cos(-x7)*sin(x9)*cos(x11) +       // dx, inertial frame
                            sin(-x7)*sin(x11)) * u1 / m;
                    _dstate.y = x4;                               // y
                    _dstate.dy = -(cos(-x7)*sin(x9)*sin(x11) -     // dy, inertial frame
                            sin(-x7)*cos(x11)) * u1 / m;
                    _dstate.z = x6;                               // z
                    _dstate.phi = x8;                               // phi
                    _dstate.dphi = l / I * u2;                       // dphi
                    _dstate.theta = x10;                              // theta
                    _dstate.dtheta = l / I * u3;                      // dtheta
                    _dstate.psi = x12;                             // psi
                    _dstate.dpsi = -l / I * u4;                     // dpsi

                    // Compute state as first temporal integral of first
                    // temporal derivative
                    x1 += _dt * _dstate.x;
                    x2 += _dt * _dstate.dx;
                    x3 += _dt * _dstate.y;
                    x4 += _dt * _dstate.dy;
                    x5 += _dt * _dstate.z;
                    x6 += _dt * _dstate.dz;
                    x7 += _dt * _dstate.phi;
                    x8 += _dt * _dstate.dphi;
                    x9 += _dt * _dstate.theta;
                    x10 += _dt * _dstate.dtheta;
                    x11 += _dt * _dstate.psi;
                    x12 += _dt * _dstate.dpsi;
                }

            } // update

            pose_t getPose()
            {
                // Negate y coordinate for rightward positive
                return pose_t {x1, -x3, x5, x7, x9, x11 };
            }

            // ---------------------------------------------------------------

        private:

            // Vehicle state (Equation 11)
            float x1;  // x
            float x2;  // dx/dt
            float x3;  // y
            float x4;  // dy/dt
            float x5;  // z
            float x6;  // dz/dt
            float x7;  // phi
            float x8;  // dphi/dt
            float x9;  // theta
            float x10; // dtheta/dt
            float x11; // psi
            float x12; // dpsi/dt

            // Vehicle state first derivative (Equation 12)
            state_t _dstate;

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

} // namespace hf
