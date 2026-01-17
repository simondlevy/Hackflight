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

#include <string.h>
#include <math.h>

#include <datatypes.h>

class Dynamics {

    public:

        static constexpr double ZMIN = 0.005;

        typedef struct {

            double x;
            double y;
            double z;
            double phi;
            double theta;
            double psi;

        } pose_t;

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

        /**
         *  World parameters
         */
        typedef struct {

            // These can be measured directly
            double g;   // gravitational constant [m/s/s]
            double rho; // air density [kg/m^3]

        } world_params_t; 

        Dynamics(
                const vehicle_params_t & vparams,
                const world_params_t & wparams,
                const double dt)
        {
            init(vparams, wparams, dt);
        }

        Dynamics(const vehicle_params_t & vparams, const double dt)
        {
            const world_params_t wparams = { 9.807, 1.225 };

            init(vparams, wparams, dt);
        }

        void setPose(const pose_t & pose)
        {
            _state.x = pose.x;
            _state.y = pose.y;
            _state.z = pose.z;
            _state.phi = pose.phi;
            _state.theta = pose.theta;
            _state.psi = pose.psi;
        }

        pose_t getPose()
        {
            return pose_t {
                _state.x, _state.y, _state.z, _state.phi, _state.theta, _state.psi
            };
        }

        vehicleState_t getVehicleStateDegrees()
        {
            return vehicleState_t {
                _state.x, _state.dx, _state.y, _state.dy, _state.z, _state.dz,
                    RAD2DEG * _state.phi,
                    RAD2DEG * _state.dphi,
                    RAD2DEG * _state.theta,
                    RAD2DEG * _state.dtheta,
                    RAD2DEG * _state.psi,
                    RAD2DEG * _state.dpsi
            };
        }

        /**
         * Sets motor spins
         */
        void update(
                const float * rpms, // single-precision for compat with mixer
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

            double u1=0, u2=0, u3=0, u4=0;

            for (unsigned int i = 0; i < rotorCount; ++i) {

                // RPM => rad/sec
                const auto omega = rpms[i] * 2 * M_PI / 60;

                // Thrust is squared rad/sec scaled by air density
                const auto omega2 = _wparams.rho * omega * omega; 

                // Multiply by thrust coefficient
                u1 += b * omega2;                  

                u2 += b * omega2 * roll[i];

                u3 += b * omega2 * pitch[i];

                u4 += d * omega2 * -yaw[i];
            }

            // Equation 12 line 6 for dz/dt in inertial (earth) frame
            _dstate.dz =
                -_wparams.g + (cos(_state.phi)*cos(_state.theta)) / m * u1;

            // We're airborne once net Z acceleration becomes positive
            if (_dstate.dz > 0) {
                _airborne = true;
            }

            // We're no longer airborne if we're descending and drop below
            // minimum altitude
            if (_airborne && _state.dz < 0 && _state.z < ZMIN) {
                _airborne = false;
                reset();
            }

            // Once airborne, we can update dynamics
            if (_airborne) {

                const auto phi = _state.phi;
                const auto theta = _state.theta;
                const auto psi = _state.psi;

                // Equation 12 : Note negations to support roll-right
                // positive

                _dstate.x = _state.dx;

                // Rotate dx/dt from body frame into inertial frame
                _dstate.dx =(cos(-phi)*sin(theta)*cos(psi) +
                        sin(-phi)*sin(psi)) * u1 / m;

                _dstate.y = _state.dy;                              

                // Rotate dy/dt from body frame into inertial frame
                _dstate.dy = (cos(-phi)*sin(theta)*sin(psi) -
                        sin(-phi)*cos(psi)) * u1 / m;

                _dstate.z = _state.dz;                             
                _dstate.phi = _state.dphi;                              
                _dstate.dphi = l / I * u2;                      
                _dstate.theta = _state.dtheta;                  
                _dstate.dtheta = l / I * u3;                  
                _dstate.psi = _state.dpsi;                   
                _dstate.dpsi = -l / I * u4;                  

                // Compute state as first temporal integral of first
                // temporal derivative
                _state.x += _dt * _dstate.x;
                _state.dx += _dt * _dstate.dx;
                _state.y += _dt * _dstate.y;
                _state.dy += _dt * _dstate.dy;
                _state.z += _dt * _dstate.z;
                _state.dz += _dt * _dstate.dz;
                _state.phi += _dt * _dstate.phi;
                _state.dphi += _dt * _dstate.dphi;
                _state.theta += _dt * _dstate.theta;
                _state.dtheta += _dt * _dstate.dtheta;
                _state.psi += _dt * _dstate.psi;
                _state.dpsi += _dt * _dstate.dpsi;

                // Keep yaw angle in [-2Pi, +2Pi]
                if(_state.psi > 2*M_PI) {
                    _state.psi -= 2*M_PI;
                }
                if(_state.psi < -2*M_PI) {
                    _state.psi += 2*M_PI;
                }
             }

        } // update

        void reset()
        {
            _state.dx = 0;
            _state.dy = 0;
            _state.z = ZMIN;
            _state.dz = 0;
            _state.dphi = 0;
            _state.dtheta = 0;
            _state.dpsi = 0;

            memset(&_dstate, 0, sizeof(vehicleState_t));
        }

        // ---------------------------------------------------------------

    private:

        static constexpr float RAD2DEG = 180.0f / M_PI;

        // Vehicle state (Equation 11)
        vehicleState_t _state;

        // Vehicle state first derivative (Equation 12)
        vehicleState_t _dstate;

        double _dt;

        vehicle_params_t _vparams;

        world_params_t _wparams;

        // Flag for whether we're airborne and can update dynamics
        bool _airborne = false;

        void init(
                const vehicle_params_t & vparams,
                const world_params_t & wparams,
                const double dt)
        {
            memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

            memcpy(&_wparams, &wparams, sizeof(world_params_t));

            _dt = dt;

            _airborne = false;
        }

}; // class Dynamics
