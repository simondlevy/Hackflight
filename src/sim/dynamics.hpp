/*
   Multirotor Dynamics class

   Based on:

   @inproceedings{DBLP:conf/icra/BouabdallahMS04,
   author    = {Samir Bouabdallah and Pierpaolo Murrieri and
   Roland Siegwart},
   title     = {Design and Control of an Indoor Micro Quadrotor},
   booktitle = {Proceedings of the 2004 {IEEE} International Conference on
   Robotics and Automation, {ICRA} 2004, April 26 - May 1, 2004,
   New Orleans, LA, {USA}},
   pages     = {4393--4398},
   year      = {2004},
   crossref  = {DBLP:conf/icra/2004},
   url       = {https://doi.org/10.1109/ROBOT.2004.1302409},
   doi       = {10.1109/ROBOT.2004.1302409},
   timestamp = {Sun, 04 Jun 2017 01:00:00 +0200},
   biburl    = {https://dblp.org/rec/bib/conf/icra/BouabdallahMS04},
   bibsource = {dblp computer science bibliography, https://dblp.org}
   }

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.

 */

#pragma once

#include <math.h>
#include <string.h>

#include <hackflight.hpp>
#include <mixer.hpp>

namespace hf {

    typedef struct {

        // Estimated
        float B; // force constant [F=b*w^2]
        float D; // torque constant [T=d*w^2]

        // These agree with values in .proto file
        float M;  // mass [kg]
        float L;  // arm length [m]

        // Estimated
        float Ix; // [kg*m^2]
        float Iy; // [kg*m^2]
        float Iz; // [kg*m^2]
        float Jr; // prop inertial [kg*m^2]

    } vehicle_params_t;


    class Dynamics {

        /*
           Dynamics class for quad-X frames using Crazyflie motor layout

           4cw   1ccw

           ^

           3ccw  2cw
         */ 

        private:

            typedef enum {

                STATUS_CRASHED,
                STATUS_LANDED,
                STATUS_AIRBORNE

            } status_e;

            // Safe landing criteria
            static constexpr float LANDING_VEL_X = 2.0;
            static constexpr float LANDING_VEL_Y = 1.0;
            static constexpr float LANDING_ANGLE = M_PI/4;

            // Graviational constant
            static constexpr float G = 9.80665;

            // Vehicle parameters [see Bouabdallah et al. 2004]
            vehicle_params_t _params;

            // Time constant
            float _dt;

            // First deriviative of state vector
            state_t _dstate;

            // Flight status
            status_e _status;

            axis3_t _inertialAccel;

        public:

            state_t state;

            Dynamics(const vehicle_params_t & params, const float dt)
            {
                memcpy(&_params, &params, sizeof(vehicle_params_t));

                _dt = dt;

                // Always start at location (0,0,0) with zero velocities
                memset(&state, 0, sizeof(state_t)); 
                memset(&_dstate, 0, sizeof(state_t));

                // Start on ground
                _status = STATUS_LANDED;

                // Initialize inertial frame acceleration in NED coordinates
                bodyZToInertial(-G, 0, 0, 0, _inertialAccel);
            }

            /*
               Implements Equations 6 and 12 from Bouabdallah et al. (2004)
               @param omegas motor speeds in radians per second
             */

            void setMotors(Mixer & mixer, const float * motors)
            {
                // Compute individual motor thrusts as air density times square of
                // motor speed
                const float omegasqr[4] = {
                    sqr(motors[0]),
                    sqr(motors[1]),
                    sqr(motors[2]),
                    sqr(motors[3])
                };

                // Compute thrust
                const auto u1 = _params.B *
                    (omegasqr[0] + omegasqr[1] + omegasqr[2] + omegasqr[3]);

                // Compute angular forces
                const auto u2 = _params.L * _params.B * mixer.roll(omegasqr);
                const auto u3 = _params.L * _params.B * mixer.pitch(omegasqr);
                const auto u4 = _params.D * mixer.yaw(omegasqr);

                // Ignore Omega ("disturbance") part of Equation 6 for now
                const float Omega = 0;

                // Use the current Euler angles to rotate the orthogonal thrust
                // vector into the inertial frame.  
                axis3_t accelENU = {};
                bodyZToInertial(u1 / _params.M,
                        state.phi, state.theta, state.psi,
                        accelENU);

                // Compute net vertical acceleration by subtracting gravity
                const auto netz = accelENU.z - G;

                // If we're not airborne, we become airborne when upward
                // acceleration has become positive
                if (_status == STATUS_LANDED) {

                    if (netz > 0) {
                        _status = STATUS_AIRBORNE;
                    }
                }

                // Once airborne, we can update dynamics
                else if (_status == STATUS_AIRBORNE) {

                    // If we've descended to the ground
                    if (state.z <= 0 and state.dz <= 0) {

                        // Big angles indicate a crash
                        const auto phi = state.phi;
                        const auto velx = state.dy;
                        const auto vely = state.dz;
                        if ((vely > LANDING_VEL_Y ||
                                    fabs(velx) > LANDING_VEL_X ||
                                    fabs(phi) > LANDING_ANGLE)) {
                            _status = STATUS_CRASHED;
                        }
                        else {
                            _status = STATUS_LANDED;
                        }

                        state.z = 0;
                        state.dz = 0;
                    }

                    // Compute the state derivatives using Equation 12
                    computeStateDerivative(accelENU, netz, u2, u3, u4, Omega);

                    // Compute state as first temporal integral of first temporal
                    // derivative
                    state.x      += _dt * _dstate.x;
                    state.dx     += _dt * _dstate.dx;
                    state.y      += _dt * _dstate.y;
                    state.dy     += _dt * _dstate.dy;
                    state.z      += _dt * _dstate.z;
                    state.dz     += _dt * _dstate.dz;
                    state.phi    += _dt * _dstate.phi;
                    state.dphi   += _dt * _dstate.dphi;
                    state.theta  += _dt * _dstate.theta;
                    state.dtheta += _dt * _dstate.dtheta;
                    state.psi    += _dt * _dstate.psi;
                    state.dpsi   += _dt * _dstate.dpsi;

                    // Once airborne, inertial-frame acceleration is same as NED
                    // acceleration
                    memcpy(&_inertialAccel, &accelENU, sizeof(axis3_t));
                }
            }

            /*
               Sets the vehicle position to the values specified in a sequence
             */
            void setPosition(const float x, const float y, const float z)
            {
                memset(&state, 0, sizeof(state_t));

                state.x = x;
                state.y = y;
                state.z = z;

                _status = z > 0 ? STATUS_AIRBORNE : STATUS_LANDED;
            }

        private:

            /*
               Implements Equation 12 computing temporal first derivative of
               state.  Should fill _dxdt[0..11] with appropriate values.
             */
            void computeStateDerivative(
                    const axis3_t accelENU,
                    const float netz,
                    const float u2,
                    const float u3,
                    const float u4,
                    const float Omega)
            {
                const auto phidot = state.dphi;
                const auto thedot = state.dtheta;
                const auto psidot = state.dpsi;

                _dstate.x = state.dx;

                _dstate.dx = accelENU.x;

                _dstate.y = state.dy;

                _dstate.dy = accelENU.y;

                _dstate.z = state.dz;

                _dstate.dz = netz;

                _dstate.phi = phidot;

                _dstate.dphi = (
                        psidot*thedot*(_params.Iy-_params.Iz) /
                        _params.Ix-_params.Jr /
                        _params.Ix*thedot*Omega + u2 / _params.Ix);

                _dstate.theta = thedot;

                _dstate.dtheta =
                    -(psidot * phidot * (_params.Iz - _params.Ix) /
                            _params.Iy + _params.Jr / 
                            _params.Iy * phidot * Omega + u3 / _params.Iy);

                _dstate.psi = psidot;

                _dstate.dpsi = (
                        thedot*phidot*(_params.Ix-_params.Iy)/_params.Iz +
                        u4/_params.Iz);
            }

            static float sqr(const float x)
            {
                return x * x;
            }

            /*
               bodyToInertial method optimized for body X=Y=0
             */
            void bodyZToInertial(
                    const float bodyZ, 
                    const float phi,
                    const float theta,
                    const float psi,
                    axis3_t & inertial)
            {

                float cph=0;
                float cth=0;
                float cps=0;
                float sph=0;
                float sth=0;
                float sps=0;

                sincos(phi, theta, psi, cph, cth, cps, sph, sth, sps);

                inertial.x = bodyZ * sph*sps+cph*cps*sth;
                inertial.y = bodyZ * cph*sps*sth-cps*sph;
                inertial.z = bodyZ * cph*cth;
            }

            void sincos(const float phi, const float theta, const float psi,
                    float & cph, float & cth, float & cps,
                    float & sph, float & sth, float & sps)
            {
                cph = cos(phi);
                cth = cos(theta);
                cps = cos(psi);
                sph = sin(phi);
                sth = sin(theta);
                sps = sin(psi);

            }
    }; 

}
