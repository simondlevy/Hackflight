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

#include "Utils.hpp"

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

        /**
         * Position map for state vector
         */
        enum {
            STATE_X,
            STATE_X_DOT,
            STATE_Y,
            STATE_Y_DOT,
            STATE_Z,
            STATE_Z_DOT,
            STATE_PHI,
            STATE_PHI_DOT,
            STATE_THETA,
            STATE_THETA_DOT,
            STATE_PSI,
            STATE_PSI_DOT,
            STATE_SIZE
        };

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

    protected:

        vehicle_params_t _vparams;
        world_params_t _wparams;

        Dynamics(vehicle_params_t & vparams)
        {
            memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

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

        // Different for each vehicle
        virtual int8_t getRotorDirection(uint8_t i) = 0;
        virtual float getThrustCoefficient(float * motors) = 0;
        virtual void computeRollAndPitch(float * motors,
                                         float * omegas2,
                                         float & roll,
                                         float & pitch) = 0;

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

            // Compute deltaT from current time minus previous
            float dt = time - _time;

            // Implement Equation 6 -------------------------------------------

            // Radians per second of rotors, and squared radians per second
            float omegas[20] = {};
            float omegas2[20] = {};

            float u1 = 0, u4 = 0, omega = 0;
            for (unsigned int i = 0; i < 4; ++i) {

                // Convert fractional speed to radians per second
                omegas[i] = motors[i] * _vparams.maxrpm * M_PI / 30;  

                // Thrust is squared rad/sec scaled by air density
                omegas2[i] = _wparams.rho * omegas[i] * omegas[i]; 

                // Thrust coefficient is constant for fixed-pitch rotors,
                // variable for collective-pitch
                u1 += getThrustCoefficient(motors) * omegas2[i];                  

                // Newton's Third Law (action/reaction) tells us that yaw is
                // opposite to net rotor spin
                u4 += _vparams.d * omegas2[i] * -getRotorDirection(i);
                omega += omegas[i] * -getRotorDirection(i);
            }
            
            // Compute roll, pitch, yaw forces (different method for
            // fixed-pitch blades vs. variable-pitch)
            float u2 = 0, u3 = 0;
            computeRollAndPitch(motors, omegas2, u2, u3);

            // ----------------------------------------------------------------

            // Use the current Euler angles to rotate the orthogonal thrust
            // vector into the inertial frame.  Negate to use NED.
            float accelNedX = 0, accelNedY = 0, accelNedZ = 0;
            bodyZToInertial(-u1 / _vparams.m,
                    _state.phi, _state.theta, _state.psi,
                    accelNedX, accelNedY, accelNedZ);

            // We're airborne once net downward acceleration goes below zero
            float netz = accelNedZ + _wparams.g;

            bool lowagl = _airborne && agl <= 0 && netz >= 0;

            bool airborne = !_airborne && netz <=0 ? true : lowagl ? false : _airborne; 

            float dphi   = _state.dphi;
            float dtheta = _state.dtheta;
            float dpsi   = _state.dpsi;

            float Ix = _vparams.Ix;
            float Iy = _vparams.Iy;
            float Iz = _vparams.Iz;
            float Jr = _vparams.Jr;

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
