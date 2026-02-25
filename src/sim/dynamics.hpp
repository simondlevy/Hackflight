#pragma once

/*
 * Standalone multirotor flight dynamics class for simulators
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

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace hf {

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

            // From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
            // We use ENU coordinates based on 
            // https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
            // Position in meters, velocity in meters/second, angles in degrees,
            // angular velocity in degrees/second.
            class State {

                public:

                    double x;       // positive forward
                    double dx;      // positive forward
                    double y;       // positive rightward
                    double dy;      // positive rightward
                    double z;       // positive upward
                    double dz;      // positive upward
                    double phi;     // positive roll right
                    double dphi;    // positive roll right
                    double theta;   // positive nose down
                    double dtheta;  // positive nose down
                    double psi;     // positive nose right
                    double dpsi;    // positive nose right

                    State() 
                        : x(0), dx(0), y(0), dy(0), z(0), dz(0), phi(0),
                        dphi(0), theta(0), dtheta(0), psi(0), dpsi(0) {}

                    State(const pose_t & p) 
                        : x(p.x), dx(0), y(p.y), dy(0), z(p.z), dz(0),
                        phi(p.phi), dphi(0), theta(p.theta), dtheta(0),
                        psi(p.psi), dpsi(0) {}

                    State(
                            const double x, const double dx,
                            const double y, const double dy,
                            const double z, const double dz,
                            const double phi, const double dphi,
                            const double theta, const double dtheta,
                            const double psi, const double dpsi)
                        : x(x), dx(dx), y(y), dy(dy), z(z), dz(dz), phi(phi),
                        dphi(dphi), theta(theta), dtheta(dtheta), psi(psi), dpsi(dpsi) {}
             };

            // Vehicle state (Equation 11)
            State state;

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

            Dynamics() = default;

            Dynamics(const pose_t & pose) : state(pose) {}

            Dynamics(const State & state, const State & dstate,
                    const bool airborne)
                : state(state), _dstate(dstate), _airborne(airborne) {}

            static auto update(
                    const Dynamics & dyn,
                    const vehicle_params_t & vparams,
                    const double dt,
                    const double * rpms,
                    const uint8_t rotorCount,
                    const int8_t * roll,
                    const int8_t * pitch,
                    const int8_t * yaw,
                    const world_params_t & wparams=EARTH_PARAMS) -> Dynamics
            {
                const auto cphi = cos(dyn.state.phi);
                const auto cnphi = cos(-dyn.state.phi);
                const auto snphi = sin(-dyn.state.phi);
                const auto ctheta = cos(dyn.state.theta);
                const auto stheta = sin(dyn.state.theta);
                const auto cpsi = cos(dyn.state.psi);
                const auto spsi = sin(dyn.state.psi);

                const auto b = vparams.b;
                const auto d = vparams.d;
                const auto I = vparams.I;
                const auto l = vparams.l;
                const auto m = vparams.m;

                const auto s = dyn.state;
                const auto ds = dyn._dstate;

                // Equation 6 ---------------------------------------

                double u1=0, u2=0, u3=0, u4=0;

                for (unsigned int i = 0; i < rotorCount; ++i) {

                    // RPM => rad/sec
                    const auto omega = rpms[i] * 2 * M_PI / 60;

                    // Thrust is squared rad/sec scaled by air density
                    const auto omega2 = wparams.rho * omega * omega; 

                    // Multiply by thrust coefficient
                    u1 += b * omega2;                  
                    u2 += b * omega2 * roll[i];
                    u3 += b * omega2 * pitch[i];
                    u4 += d * omega2 * -yaw[i];
                }

                const auto ddz = -wparams.g + (cphi * ctheta) / m * u1;

                // Equation 12 line 6 for dz/dt in inertial (earth) frame
                const auto airborne =
                    ddz > 0 ? true :
                    dyn._airborne && s.dz < 0 && s.z < ZMIN ? false :
                    dyn._airborne;

                // Compute state as first temporal integral of first temporal
                // derivative
                const auto newstate = State(
                        s.x + (airborne ? dt * ds.x : 0),
                        s.dx + (airborne ? dt * ds.dx : 0),
                        s.y + (airborne ? dt * ds.y : 0),
                        s.dy + (airborne ? dt * ds.dy : 0),
                        s.z + (airborne ? dt * ds.z : 0),
                        s.dz + (airborne ? dt * ds.dz : 0),
                        s.phi + (airborne ? dt * ds.phi : 0),
                        s.dphi + (airborne ? dt * ds.dphi : 0),
                        s.theta + (airborne ? dt * ds.theta : 0),
                        s.dtheta + (airborne ? dt * ds.dtheta : 0),
                        constrain_psi(s.psi + (airborne ? dt * ds.psi : 0)),
                        s.dpsi + (airborne ? dt * ds.dpsi : 0));

                const auto newdstate = !airborne ? State() :

                    // Equation 12
                    State(
                            s.dx,
                            (cnphi*stheta*cpsi + snphi*spsi) * u1 / m,
                            s.dy,
                            (cnphi*stheta*spsi - snphi*cpsi) * u1 / m,
                            s.dz,
                            ddz,
                            s.dphi,
                            l / I * u2, 
                            s.dtheta,
                            l / I * u3,
                            s.dpsi,
                            -l / I * u4);

                return Dynamics(newstate, newdstate, airborne);

            } // update

            void update(
                    const double * rpms,
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

                    // Keep yaw angle in [-2Pi, +2Pi]
                    state.psi = constrain_psi(state.psi);
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

                _dstate.x = 0;
                _dstate.dx = 0;
                _dstate.y = 0;
                _dstate.dy = 0;
                _dstate.z = 0;
                _dstate.dz = 0;
                _dstate.phi = 0;
                _dstate.dphi = 0;
                _dstate.theta = 0;
                _dstate.dtheta = 0;
                _dstate.psi = 0;
                _dstate.dpsi = 0;
            }

            // ---------------------------------------------------------------

        private:

            static constexpr world_params_t EARTH_PARAMS = { 9.807, 1.225 };

            // Vehicle state first derivative (Equation 12)
            State _dstate;

            double _dt;

            vehicle_params_t _vparams;

            world_params_t _wparams;

            // Flag for whether we're airborne and can update dynamics
            bool _airborne = false;

            static float constrain_psi(const double psi)
            {
                const auto twopi = 2 * M_PI;
                return psi + (psi > twopi ? -twopi : psi < -twopi ? twopi : 0);
            }

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

} // namespace hf
