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

#include <mixer.hpp>
#include <utils.hpp>

namespace hf {

    class Dynamics {

        friend class Gyrometer;
        friend class Accelerometer;
        friend class OpticalFlow;
        friend class Rangefinder;

        public:

            typedef struct {

                float x;
                float y;
                float z;
                float phi;
                float theta;
                float psi;

            } pose_t;

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

            Dynamics(
                    const vehicle_params_t & vparams,
                    const float dt,
                    const float gravity = 9.80665e0,
                    const float air_density = 1.225)
            {
                memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

                _dt = dt;

                _rho = air_density;
                _g = gravity;

                _airborne = false;
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
                    const auto omega2 = _rho * omegas[i] * omegas[i]; 

                    // Multiply by thrust coefficient
                    u1 += b * omega2;                  

                    u2 += b * omega2 * mixer->roll(i);

                    u3 += b * omega2 * mixer->pitch(i);

                    u4 += d * omega2 * mixer->yaw(i);
                }

                // Equation 12 line 6 for dz/dt in inertial (earth) frame
                _dx6 = -_g + (cos(_x7)*cos(_x9)) * 1 / m * u1;

                // We're airborne once net Z acceleration becomes positive
                if (_dx6 > 0) {
                    _airborne = true;
                }

                // Once airborne, we can update dynamics
                if (_airborne) {

                    // Equation 12 : Note negations to support roll-right
                    // positive

                    _dx1 = _x2;                                 // x
                    _dx2 =(cos(-_x7)*sin(_x9)*cos(_x11) +       // dx, inertial frame
                            sin(-_x7)*sin(_x11)) * u1 / m;
                    _dx3 = _x4;                                 // y
                    _dx4 = -(cos(-_x7)*sin(_x9)*sin(_x11) -     // dy, inertial frame
                            sin(-_x7)*cos(_x11)) * u1 / m;
                    _dx5 = _x6;                                 // z
                    _dx7 = _x8;                                 // phi
                    _dx8 = l / I * u2;                          // dphi
                    _dx9 = _x10;                                // theta
                    _dx10 = l / I * u3;                         // dtheta
                    _dx11 = _x12;                               // psi
                    _dx12 = -l / I * u4;                        // dpsi

                    // Compute state as first temporal integral of first
                    // temporal derivative
                    _x1 += _dt * _dx1;
                    _x2 += _dt * _dx2;
                    _x3 += _dt * _dx3;
                    _x4 += _dt * _dx4;
                    _x5 += _dt * _dx5;
                    _x6 += _dt * _dx6;
                    _x7 += _dt * _dx7;
                    _x8 += _dt * _dx8;
                    _x9 += _dt * _dx9;
                    _x10 += _dt * _dx10;
                    _x11 += _dt * _dx11;
                    _x12 += _dt * _dx12;
                }

            } // update

            pose_t getPose()
            {
                // Negate y coordinate for rightward positive
                return pose_t {_x1, -_x3, _x5, _x7, _x9, _x11 };
            }

            // Vehicle state (Equation 11)
            // Should be private, but we need it in for control loop until
            // we get sensor fusion working
            float _x1;  // x
            float _x2;  // dx/dt
            float _x3;  // y
            float _x4;  // dy/dt
            float _x5;  // z
            float _x6;  // dz/dt
            float _x7;  // phi
            float _x8;  // dphi/dt
            float _x9;  // theta
            float _x10; // dtheta/dt
            float _x11; // psi
            float _x12; // dpsi/dt

            // ---------------------------------------------------------------

        private:

            float _dt;

            vehicle_params_t _vparams;

            // Vehicle state first derivative (Equation 12)
            float _dx1;  // x
            float _dx2;  // dx/dt
            float _dx3;  // y
            float _dx4;  // dy/dt
            float _dx5;  // z
            float _dx6;  // dz/dt
            float _dx7;  // phi
            float _dx8;  // dphi/dt
            float _dx9;  // theta
            float _dx10; // dtheta/dt
            float _dx11; // psi
            float _dx12; // dpsi/dt

            float _g; // gravitational constant
            float _rho; // air density

            // Flag for whether we're airborne and can update dynamics
            bool _airborne = false;

    }; // class Dynamics

} // namespace hf
