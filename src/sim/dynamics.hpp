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
#include <stdio.h>
#include <string.h>

#include <sim/datatypes.hpp>

namespace hf {

    class Dynamics {

        public:

            static constexpr float kRollPitchYawScale = 1000;

            // Vehicle state (Equation 11)
            SimState state;

            /**
             *  Vehicle parameters
             */
            typedef struct {

                // Proporitiona to motor kV
                double thrust_scale;

                // These can be measured directly
                double m;  // mass [kg]
                double l;  // arm length [m]

                // These should be estimated to get realistic behavior
                double b;  // thrust coefficient [F=b*w^2]
                double I;  // body inertia [kg*m^2]  for roll, pitch
                double d;  // drag coefficient [T=d*w^2] for yaw

            } VehicleParams; 

            /**
             *  World parameters
             */
            typedef struct {

                // These can be measured directly
                double g;   // gravitational constant [m/s/s]
                double rho; // air density [kg/m^3]

            } WorldParams; 

            Dynamics() = default;

            Dynamics& operator=(const Dynamics& other) = default;

            Dynamics(const Pose & pose)
                : state(pose), airborne_(false) {}

            Dynamics(const SimState & state, const SimState & dstate,
                    const bool airborne)
                : state(state), dstate_(dstate), airborne_(airborne) {}

            static auto Update(
                    const Dynamics & dyn,
                    const VehicleParams & vparams,
                    const float dt,
                    const Setpoint & forces,
                    const WorldParams wparams = { 9.807, 1.225 }) -> Dynamics
            {
                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const double u1 = vparams.b * wparams.rho * forces.thrust;
                const double u2 = vparams.b * wparams.rho * forces.roll;
                const double u3 = vparams.b * wparams.rho * forces.pitch;
                const double u4 = vparams.d * wparams.rho * forces.yaw;

                // -----------------------------------------------------------

                const auto cphi = cos(dyn.state.phi);
                const auto cnphi = cos(-dyn.state.phi);
                const auto snphi = sin(-dyn.state.phi);
                const auto ctheta = cos(dyn.state.theta);
                const auto stheta = sin(dyn.state.theta);
                const auto cpsi = cos(dyn.state.psi);
                const auto spsi = sin(dyn.state.psi);

                const auto I = vparams.I;
                const auto l = vparams.l;
                const auto m = vparams.m;

                const auto s = dyn.state;
                const auto ds = dyn.dstate_;

                const auto ddz = -wparams.g + (cphi * ctheta) / m * u1;

                //printf("%f,%f,%f\n", forces.thrust, u1*m, ddz);

                // Equation 12 line 6 for dz/dt in inertial (earth) frame
                const auto airborne =
                    ddz > 0 ? true :
                    dyn.airborne_ && s.dz < 0 && s.z <= 0 ? false :
                    dyn.airborne_;

                // Compute state as first temporal integral of first temporal
                // derivative
                const auto newstate = SimState(
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

                const auto newdstate = !airborne ? SimState() :

                    // Equation 12
                    SimState(
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

                return dyn.airborne_ && !airborne ? // just landed ?

                    // yes: reset dynamics, keeping current pose
                    Dynamics(Pose(s.x, s.y, s.z, s.phi, s.theta, s.psi)) :

                        // no: update dynamics as usual
                        Dynamics(newstate, newdstate, airborne); 

            } // update

            static auto RpmToOmegaSquared(const double rpm) -> double
            {
                const auto omega = rpm * 2 * M_PI / 60;

                return omega * omega;
            }

        private:

            // Vehicle state first derivative (Equation 12)
            SimState dstate_;

            // Flag for whether we're airborne and can update dynamics
            bool airborne_;

            static auto constrain_psi(const double psi) -> float
            {
                const auto twopi = 2 * M_PI;
                return psi + (psi > twopi ? -twopi : psi < -twopi ? twopi : 0);
            }

    }; // class Dynamics

} // namespace hf
