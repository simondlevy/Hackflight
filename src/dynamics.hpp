/*
 * Header-only code for flight dynamics
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#include <hackflight.hpp>

namespace hf {

    class Dynamics {

        public:

            /**
             *  Vehicle parameters
             */
            typedef struct {

                // These can be measured directly
                double m;  // mass [kg]
                double l;  // arm length [m]

                // These should be estimated to get realistic behavior
                double b;  // thrust coefficient [F=b*w^2]
                double d;  // drag coefficient [T=d*w^2]
                double I;  // body inertia [kg*m^2]; we ignore x/y/z distinction

            } vehicle_params_t; 

            Dynamics(
                    const vehicle_params_t & vparams,
                    const double dt,
                    const double gravity = 9.80665e0,
                    const double air_density = 1.225)
            {
                memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

                _dt = dt;

                _rho = air_density;
                _g = gravity;

                // Always start at location (0,0,0)
                memset(&_state, 0, sizeof(state_t));

                _airborne = false;
            }

            /**
             * Sets motor spins
             *
             * @param omegas motor spins in radians per second
             * @param dt deltaT in seconds
             */
            void update(const float * omegas, Mixer * mixer) 
            {
                // Equation 6 ---------------------------------------

                double u1 = 0, u2 = 0, u3 = 0, u4 = 0;

                for (unsigned int i = 0; i < mixer->rotorCount(); ++i) {

                    // Thrust is squared rad/sec scaled by air density
                    const auto omega2 = _rho * omegas[i] * omegas[i]; 

                    // Multiply by thrust coefficient
                    u1 += _vparams.b * omega2;                  

                    u2 += _vparams.b * omega2 * mixer->roll(i);

                    u3 += _vparams.b * omega2 * mixer->pitch(i);

                    // Newton's Third Law (action/reaction) tells us that yaw
                    // is opposite to net rotor spin
                    u4 -= _vparams.d * omega2 * mixer->yaw(i);
                }


                // Use the current Euler angles to rotate the orthogonal thrust
                // vector into the inertial frame
                double euler[3] = {_state.phi, _state.theta, _state.psi};
                double accelENU[3] = {};
                bodyZToInertial(u1 / _vparams.m, euler, accelENU);

                // Subtact gravity from thrust to get net vertical acceleration
                double netz = accelENU[2] - _g;

                // We're airborne once net Z acceleration becomes positive
                if (netz > 0) {
                    _airborne = true;
                }

                // Once airborne, we can update dynamics
                if (_airborne) {

                    const auto phidot = _state.dphi;
                    const auto thedot = _state.dtheta;
                    const auto psidot = _state.dpsi;

                    const auto I = _vparams.I;
                    const auto l = _vparams.l;

                    // Equation 12 --------------------------------------------

                    // x'
                    state_deriv.x = _state.dx;
                    const auto dx1 = _x2;

                    // x''
                    state_deriv.dx = accelENU[0];
                    const auto dx2 = accelENU[0];

                    // y'
                    state_deriv.y = _state.dy;
                    const auto dx3 = _x4;

                    // y''
                    state_deriv.dy = accelENU[1];
                    const auto dx4 = accelENU[1];

                    // z'
                    state_deriv.z = _state.dz;
                    const auto dx5 = _x6;

                    // z''
                    state_deriv.dz = netz;
                    const auto dx6 = netz;

                    // phi'
                    state_deriv.phi = phidot;
                    const auto dx7 = _x8;

                    // phi''
                    state_deriv.dphi =  l / I * u2;
                    const auto dx8 = l / I * u2;

                    // theta'
                    state_deriv.theta = thedot;
                    const auto dx9 = _x10;

                    // theta''
                    state_deriv.dtheta = l / I * u3;
                    const auto dx10 = l / I * u3;

                    // psi'
                    state_deriv.psi = psidot;
                    const auto dx11 = _x12;

                    // psi''
                    state_deriv.dpsi = l / I * u4;
                    const auto dx12 = l / I * u4;

                    // Compute state as first temporal integral of first
                    // temporal derivative
                    _state.x += _dt * state_deriv.x;
                    _state.dx += _dt * state_deriv.dx;
                    _state.y += _dt * state_deriv.y;
                    _state.dy += _dt * state_deriv.dy;
                    _state.z += _dt * state_deriv.z;
                    _state.dz += _dt * state_deriv.dz;
                    _state.phi += _dt * state_deriv.phi;
                    _state.dphi += _dt * state_deriv.dphi;
                    _state.theta += _dt * state_deriv.theta;
                    _state.dtheta += _dt * state_deriv.dtheta;
                    _state.psi += _dt * state_deriv.psi;
                    _state.dpsi += _dt * state_deriv.dpsi;

                    _x1 += _dt * dx1;
                    _x2 += _dt * dx2;
                    _x3 += _dt * dx3;
                    _x4 += _dt * dx4;
                    _x5 += _dt * dx5;
                    _x6 += _dt * dx6;
                    _x7 += _dt * dx7;
                    _x8 += _dt * dx8;
                    _x9 += _dt * dx9;
                    _x10 += _dt * dx10;
                    _x11 += _dt * dx11;
                    _x12 += _dt * dx12;
                }

            } // update

            state_t getState() 
            {
                return state_t {
                    (float)_x1,
                        (float)_x2,
                        -(float)_x3,
                        -(float)_x4, // negate for rightward positive
                        (float)_x5,
                        (float)_x6,
                        Utils::RAD2DEG *(float)_x7,
                        Utils::RAD2DEG *(float)_x8,
                        Utils::RAD2DEG *(float)_x9,
                        Utils::RAD2DEG *(float)_x10,
                        Utils::RAD2DEG *(float)_x11,
                        Utils::RAD2DEG *(float)_x12
                };

                /*
                 return state_t {
                    _state.x,
                        _state.dx,
                        -_state.y,
                        -_state.dy, // negate for rightward positive
                        _state.z,
                        _state.dz,
                        Utils::RAD2DEG *_state.phi,
                        Utils::RAD2DEG *_state.dphi,
                        Utils::RAD2DEG *_state.theta,
                        Utils::RAD2DEG *_state.dtheta,
                        Utils::RAD2DEG *_state.psi,
                        Utils::RAD2DEG *_state.dpsi
                };*/
            }

        private:

            double _dt;

            vehicle_params_t _vparams;

            state_t _state;

            double _x1;
            double _x2;
            double _x3;
            double _x4;
            double _x5;
            double _x6;
            double _x7;
            double _x8;
            double _x9;
            double _x10;
            double _x11;
            double _x12;

            double _g; // gravitational constant
            double _rho; // air density

            state_t state_deriv;

            // Flag for whether we're airborne and can update dynamics
            bool _airborne = false;

            // y = Ax + b helper for frame-of-reference conversion methods
            static void dot(double A[3][3], double x[3], double y[3])
            {
                for (uint8_t j = 0; j < 3; ++j) {
                    y[j] = 0;
                    for (uint8_t k = 0; k < 3; ++k) {
                        y[j] += A[j][k] * x[k];
                    }
                }
            }

            // bodyToInertial method optimized for body X=Y=0
            static void bodyZToInertial(
                    const double bodyZ,
                    const double rotation[3],
                    double inertial[3])
            {
                double phi = rotation[0];
                double theta = rotation[1];
                double psi = rotation[2];

                double cph = cos(phi);
                double sph = sin(phi);
                double cth = cos(theta);
                double sth = sin(theta);
                double cps = cos(psi);
                double sps = sin(psi);

                // This is the rightmost column of the body-to-inertial rotation
                // matrix
                double R[3] = {
                    sph * sps + cph * cps * sth,
                    cph * sps * sth - cps * sph,
                    cph * cth };

                for (uint8_t i = 0; i < 3; ++i) {
                    inertial[i] = bodyZ * R[i];
                }
            }

    }; // class Dynamics

} // namespace hf
