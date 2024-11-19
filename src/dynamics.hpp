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

                // ------------------------------------------------------------

                // Use the current Euler angles to rotate the orthogonal thrust
                // vector into the inertial frame
                double euler[3] = {_state.phi, _state.theta, _state.psi};
                double accelENU[3] = {};
                bodyZToInertial(u1 / _vparams.m, euler, accelENU);

                static long count;
                if (count++ % 100 == 0) {
                    //printf("accelENU[1 = %+3.3f\n", accelENU[1]);
                }

                // Subtact gravity from thrust to get net vertical acceleration
                double netz = accelENU[2] - _g;

                // We're airborne once net Z acceleration becomes positive
                if (netz > 0) {
                    _airborne = true;
                }

                // Once airborne, we can update dynamics
                if (_airborne) {

                    // Compute the state derivatives using Equation 12
                    computeStateDerivative(accelENU, netz, u2, u3, u4);

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
                }

            } // update

            state_t getState() 
            {
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
                };
            }

        private:

            double _dt;

            vehicle_params_t _vparams;

            state_t _state;

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

            /**
             * Implements Equation 12 computing temporal first derivative of
             * state.  Should fill _dxdx[0..11] with appropriate values.
             *
             * @param accelENU acceleration in ENU inertial frame
             * @param netz accelENU[2] with gravitational constant added in
             * @param omega net torque from rotors
             * @param u2 roll force
             * @param u3 pitch force
             * @param u4 yaw force
             */
            void computeStateDerivative(
                    const double accelENU[3],
                    const double netz,
                    const double u2,
                    const double u3,
                    const double u4)
            {
                const auto phidot = _state.dphi;
                const auto thedot = _state.dtheta;
                const auto psidot = _state.dpsi;

                const auto I = _vparams.I;
                const auto l = _vparams.l;

                // x'
                state_deriv.x = _state.dx;

                // x''
                state_deriv.dx = accelENU[0];

                // y'
                state_deriv.y = _state.dy;

                // y''
                state_deriv.dy = accelENU[1];

                // z'
                state_deriv.z = _state.dz;

                // z''
                state_deriv.dz = netz;

                // phi'
                state_deriv.phi = phidot;

                // phi''
                state_deriv.dphi =  l / I * u2;

                // theta'
                state_deriv.theta = thedot;

                // theta''
                state_deriv.dtheta = l / I * u3;

                // psi'
                state_deriv.psi = psidot;

                // psi''
                state_deriv.dpsi = l / I * u4;
            }

    }; // class Dynamics

} // namespace hf
