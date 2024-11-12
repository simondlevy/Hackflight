/*
 * Header-only code for flight dynamics
 *
 * Based on:
 *
 *   @inproceedings{DBLP:conf/icra/BouabdallahMS04,
 *     author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland
 Siegwart},
 *     title     = {Design and Control of an Indoor Micro Quadrotor},
 *     booktitle = {Proceedings of the 2004 {IEEE} International Conference on
 Robotics and Automation, {ICRA} 2004, April 26 - May 1,
 2004, New Orleans, LA, {USA}},
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
 * Copyright (C) 2024 Simon D. Levy
 *
 * MIT License
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

            state_t state;

            /**
             *  Vehicle parameters
             */
            typedef struct {

                double b;  // thrust coefficient [F=b*w^2]
                double l;  // arm length [m]

                double d;  // drag coefficient [T=d*w^2]
                double m;  // mass [kg]
                double Ix; // [kg*m^2] 
                double Iy; // [kg*m^2] 
                double Iz; // [kg*m^2] 
                double Jr; // rotor inertial [kg*m^2] 

            } vehicle_params_t; 

            Dynamics(
                    const vehicle_params_t & vparams,
                    const double gravity = 9.80665,
                    const double air_density = 1.225)
            {
                memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

                _rho = air_density;
                _g = gravity;
            }        

            /**
             * Initializes kinematic pose, with flag for whether we're airbone
             * (helps with testing gravity).
             *
             * @param rotation initial rotation
             * @param airborne allows us to start on the ground (default) or in the
             * air (e.g., gravity test)
             */
            void init(const double rotation[3], const bool airborne = false)
            {
                // Always start at location (0,0,0)
                memset(&state, 0, sizeof(state));

                state.phi   = rotation[0];
                state.theta = rotation[1];
                state.psi   = rotation[2];

                _airborne = airborne;

                // Initialize inertial frame acceleration in NED coordinates
                bodyZToInertial(-_g, rotation, _inertialAccel);

                // We usuall start on ground, but can start in air for testing
                _airborne = airborne;
            }

            /**
             * Sets motor spins
             *
             * @param motor spins in radians per second
             */
            void update(const float * fomegas, const double dt) 
            {
                double omegas[MAX_ROTORS] = {};

                // Convert motor values to double-precision for consistency
                for (auto k=0; k<_rotorCount; ++k) {
                    omegas[k] = fomegas[k];
                }

                // Implement Equation 6 -------------------------------------------

                // Radians per second of rotors, and squared radians per second
                double omegas2[MAX_ROTORS] = {};

                double u1 = 0, u4 = 0, omega = 0;
                for (unsigned int i = 0; i < _rotorCount; ++i) {

                    // Thrust is squared rad/sec scaled by air density
                    omegas2[i] = _rho * omegas[i] * omegas[i]; 

                    // Multiply by thrust coefficient
                    u1 += _vparams.b * omegas2[i];                  

                    // Newton's Third Law (action/reaction) tells us that yaw is
                    // opposite to net rotor spin
                    //u4 += _vparams.d * omegas2[i] * -getRotorDirection(i);
                    //omega += omegas[i] * -getRotorDirection(i);
                }

                // Compute roll, pitch, yaw forces (different method for
                // fixed-pitch blades vs. variable-pitch)
                double u2 = 0, u3 = 0;
                //computeRollAndPitch(omegas, omegas2, u2, u3);

                // ----------------------------------------------------------------

                // Use the current Euler angles to rotate the orthogonal thrust
                // vector into the inertial frame
                double euler[3] = {state.phi, state.theta, state.psi};
                double accelNED[3] = {};
                bodyZToInertial(u1 / _vparams.m, euler, accelNED);

                // Subtact gravity from thrust to get net vertical acceleration
                double netz = accelNED[2] - _g;

                _airborne = netz > 0;

                // Once airborne, we can update dynamics
                if (_airborne) {

                    // Compute the state derivatives using Equation 12
                    computeStateDerivative(accelNED, netz, omega, u2, u3, u4);

                    // Compute state as first temporal integral of first temporal
                    // derivative
                    state.x += dt * state_deriv.x;
                    state.dx += dt * state_deriv.dx;
                    state.y += dt * state_deriv.y;
                    state.dy += dt * state_deriv.dy;
                    state.z += dt * state_deriv.z;
                    state.dz += dt * state_deriv.dz;
                    state.phi += dt * state_deriv.phi;
                    state.dphi += dt * state_deriv.dphi;
                    state.theta += dt * state_deriv.theta;
                    state.dtheta += dt * state_deriv.dtheta;
                    state.psi += dt * state_deriv.psi;
                    state.dpsi += dt * state_deriv.dpsi;

                    // Once airborne, inertial-frame acceleration is same as NED
                    // acceleration
                    _inertialAccel[0] = accelNED[0];
                    _inertialAccel[1] = accelNED[1];
                    _inertialAccel[2] = accelNED[2];
                }

            } // update

        private:

            // arbitrary; avoids dynamic allocation
            static const uint8_t MAX_ROTORS = 20; 

            vehicle_params_t _vparams;

            double _g; // gravitational constant
            double _rho; // air density

            state_t state_deriv;

            // Flag for whether we're airborne and can update dynamics
            bool _airborne = false;

            // Inertial-frame acceleration
            double _inertialAccel[3] = {};

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
                double R[3] = { sph * sps + cph * cps * sth,
                    cph * sps * sth - cps * sph,
                    cph * cth };

                for (uint8_t i = 0; i < 3; ++i) {
                    inertial[i] = bodyZ * R[i];
                }
            }

            // quad, hexa, octo, etc.
            uint8_t _rotorCount = 4;

            /**
             * Implements Equation 12 computing temporal first derivative of state.
             * Should fill _dxdx[0..11] with appropriate values.
             * @param accelNED acceleration in NED inertial frame
             * @param netz accelNED[2] with gravitational constant added in
             * @param omega net torque from rotors
             * @param u2 roll force
             * @param u3 pitch force
             * @param u4 yaw force
             */
            void computeStateDerivative(double accelNED[3],
                    double netz,
                    double omega,
                    double u2,
                    double u3,
                    double u4)
            {
                double phidot = state.dphi;
                double thedot = state.dtheta;
                double psidot = state.dpsi;

                double Ix = _vparams.Ix;
                double Iy = _vparams.Iy;
                double Iz = _vparams.Iz;
                double Jr = _vparams.Jr;

                // x'
                state_deriv.x = state.dx;

                // x''
                state_deriv.dx = accelNED[0];

                // y'
                state_deriv.y = state.dy;

                // y''
                state_deriv.dy = accelNED[1];

                // z'
                state_deriv.z = state.dz;

                // z''
                state_deriv.dz = netz;

                // phi'
                state_deriv.phi = phidot;

                // phi''
                state_deriv.dphi = psidot * thedot * (Iy - Iz) / Ix - Jr / 
                    Ix * thedot * omega + u2 / Ix;

                // theta'
                state_deriv.theta = thedot;

                // theta''
                state_deriv.dtheta = -(psidot * phidot * (Iz - Ix) / Iy + Jr / 
                        Iy * phidot * omega + u3 / Iy);

                // psi'
                state_deriv.psi = psidot;

                // psi''
                state_deriv.dpsi = thedot * phidot * (Ix - Iy) / Iz + u4 / Iz;
            }


    }; // class Dynamics

} // namespace hf
