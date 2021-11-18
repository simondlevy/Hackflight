/*
 * Header-only code for platform-independent flight dynamics
 *
 * Should work for any simulator, vehicle, or operating system
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
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

class Dynamics {

    private:

        typedef struct {

            float g;  // gravitational constant
            float rho;  // air density

        } world_params_t; 

        world_params_t EARTH_PARAMS = { 
            9.80665,  // g graviational constant
            1.225 // rho air density 
        };

    public:

        /**
         *  Vehicle parameters
         */
        typedef struct {

            float d;  // drag coefficient [T=d*w^2]
            float m;  // mass [kg]
            float Ix; // [kg*m^2] 
            float Iy; // [kg*m^2] 
            float Iz; // [kg*m^2] 
            float Jr; // rotor inertial [kg*m^2] 
            uint16_t maxrpm; // maxrpm

        } vehicle_params_t; 

        typedef struct {

            float b;  // thrust coefficient [F=b*w^2]
            float l;  // arm length [m]

        } fixed_pitch_params_t; 

        typedef struct {

            float x;
            float dx;
            float y;
            float dy;
            float z;
            float dz;
            float phi;
            float dphi;
            float theta;
            float dtheta;
            float psi;
            float dpsi;

        } state_t;

        vehicle_params_t _vparams;
        world_params_t _wparams;
        fixed_pitch_params_t _fpparams;

        Dynamics(vehicle_params_t & vparams, fixed_pitch_params_t & fpparams)
        {
            memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

            memcpy(&_fpparams, &fpparams, sizeof(fixed_pitch_params_t));

            // Default to Earth params (can be overridden by setWorldParams())
            memcpy(&_wparams, &EARTH_PARAMS, sizeof(world_params_t));
        }        

        // bodyToInertial method optimized for body X=Y=0
        static void bodyZToInertial(float bodyZ,
                                    const float phi, const float theta, const float psi,
                                    float & inertialX, float & inertialY, float & inertialZ)
        {
            float cph = cos(phi);
            float sph = sin(phi);
            float cth = cos(theta);
            float sth = sin(theta);
            float cps = cos(psi);
            float sps = sin(psi);

            // This is the rightmost column of the body-to-inertial rotation matrix
            inertialX = bodyZ * (sph * sps + cph * cps * sth);
            inertialY = bodyZ * (cph * sps * sth - cps * sph);
            inertialZ = bodyZ * (cph * cth);
        }

        static float integrate(float val, float dval, float dt, bool airborne, bool lowagl)
        {
            return lowagl ? 0 : val + dt * (airborne ? dval : 0);
        }

    public:

        /**
         * Updates state.
         */
        void update(float * motors, state_t & state, float agl, float time) 
        {
            // Local state
            static state_t _state;
            static float _time;
            static bool _airborne;

            // Parameter abbreviations
            float Ix     = _vparams.Ix;
            float Iy     = _vparams.Iy;
            float Iz     = _vparams.Iz;
            float Jr     = _vparams.Jr;
            float m      = _vparams.m;
            float d      = _vparams.d;
            float maxrpm = _vparams.maxrpm;
            float g      = _wparams.g;
            float rho    = _wparams.rho;
            float b      = _fpparams.b;
            float l      = _fpparams.l;

            // State abbreviations
            float dphi   = _state.dphi;
            float dtheta = _state.dtheta;
            float dpsi   = _state.dpsi;

            // Compute deltaT from current time minus previous
            float dt = time - _time;

            printf("cpp dt: %f", dt);

            // Convert fractional speed to radians per second
            float omegas_m1 = motors[0] * maxrpm * M_PI / 30;
            float omegas_m2 = motors[1] * maxrpm * M_PI / 30;
            float omegas_m3 = motors[2] * maxrpm * M_PI / 30;
            float omegas_m4 = motors[3] * maxrpm * M_PI / 30;

            // Thrust is squared rad/sec scaled by air density
            float omegas2_m1 = rho * omegas_m1 * omegas_m1;
            float omegas2_m2 = rho * omegas_m2 * omegas_m2;
            float omegas2_m3 = rho * omegas_m3 * omegas_m3;
            float omegas2_m4 = rho * omegas_m4 * omegas_m4;

            // Newton's Third Law (action/reaction) tells us that yaw is
            // opposite to net rotor spin
            float omega = omegas_m1 + omegas_m2 - omegas_m3 - omegas_m4;

             // Implement Equation 6 -------------------------------------------
            float u1 = b * (omegas2_m1 + omegas2_m2 + omegas2_m3 + omegas2_m4);
            float u2 = l * b * (-omegas2_m1 + omegas2_m2 + omegas2_m3 - omegas2_m4);
            float u3 = l * b * (-omegas2_m1 + omegas2_m2 - omegas2_m3 + omegas2_m4);
            float u4 = b *     ( omegas2_m1 + omegas2_m2 - omegas2_m3 - omegas2_m4);

            // Use the current Euler angles to rotate the orthogonal thrust
            // vector into the inertial frame.  Negate to use NED.
            float accelNedX = 0, accelNedY = 0, accelNedZ = 0;
            bodyZToInertial(-u1 / m,
                    _state.phi, _state.theta, _state.psi,
                    accelNedX, accelNedY, accelNedZ);

            // We're airborne once net downward acceleration goes below zero
            float netz = accelNedZ + g;

            bool lowagl = _airborne && agl <= 0 && netz >= 0;

            bool airborne = !_airborne && netz <=0 ? true : lowagl ? false : _airborne; 

            // Apply Equation 5 to get second derivatives of Euler angles
            float ddphi   = dpsi * dtheta *(Iy - Iz) / Ix - Jr / Ix * dtheta * omega + u2 / Ix;
            float ddtheta = dpsi * dphi * (Iz - Ix) / Iy + Jr / Iy * dphi * omega + u3 / Iy;
            float ddpsi   = dtheta * dphi * (Ix - Iy) / Iz + u4 / Iz; 

            // Compute the state derivatives using Equation 12, and integrate them
            // to get the updated state

            state.x      = integrate(_state.x,  _state.dx, dt, airborne, lowagl);
            state.dx     = integrate(_state.dx, accelNedX, dt, airborne, lowagl);
            state.y      = integrate(_state.y,  _state.dy, dt, airborne, lowagl);
            state.dy     = integrate(_state.dy, accelNedY, dt, airborne, lowagl);

            // Special Z-axis handling for low AGL
            state.z      = _state.z + (lowagl ? agl : 0) + dt * (_airborne ? _state.dz : 5 * agl);

            state.dz     = integrate(_state.dz,     netz,          dt, airborne, lowagl);
            state.phi    = integrate(_state.phi,    _state.dphi,   dt, airborne, lowagl);
            state.dphi   = integrate(_state.dphi,   ddphi,         dt, airborne, lowagl);
            state.theta  = integrate(_state.theta,  _state.dtheta, dt, airborne, lowagl);
            state.dtheta = integrate(_state.dtheta, -ddtheta,      dt, airborne, lowagl);
            state.psi    = integrate(_state.psi,    _state.dpsi,   dt, airborne, lowagl);
            state.dpsi   = integrate(_state.dpsi,   ddpsi,         dt, airborne, lowagl);
       
            // Maintain state between calls
            memcpy(&_state, &state, sizeof(state_t));
            _time = time;
            _airborne = airborne;

        } // update

}; // class Dynamics
