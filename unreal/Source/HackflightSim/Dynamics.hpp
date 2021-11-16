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

        // y = Ax + b helper for frame-of-reference conversion methods
        static void dot(float A[3][3], float x[3], float y[3])
        {
            for (uint8_t j = 0; j < 3; ++j) {
                y[j] = 0;
                for (uint8_t k = 0; k < 3; ++k) {
                    y[j] += A[j][k] * x[k];
                }
            }
        }

        // bodyToInertial method optimized for body X=Y=0
        static void bodyZToInertial(float bodyZ,
                                    const float rotation[3],
                                    float inertial[3])
        {
            float phi = rotation[0];
            float theta = rotation[1];
            float psi = rotation[2];

            float cph = cos(phi);
            float sph = sin(phi);
            float cth = cos(theta);
            float sth = sin(theta);
            float cps = cos(psi);
            float sps = sin(psi);

            // This is the rightmost column of the body-to-inertial rotation
            // matrix
            float R[3] = { sph * sps + cph * cps * sth,
                cph * sps * sth - cps * sph,
                cph * cth };

            for (uint8_t i = 0; i < 3; ++i) {
                inertial[i] = bodyZ * R[i];
            }
        }

        // Different for each vehicle
        virtual int8_t getRotorDirection(uint8_t i) = 0;
        virtual float getThrustCoefficient(float * motors) = 0;
        virtual void computeRollAndPitch(float * motors,
                                         float * omegas2,
                                         float & roll,
                                         float & pitch) = 0;

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
            float euler[3] = { _state.phi, _state.theta, _state.psi };
            float accelNED[3] = {};
            bodyZToInertial(-u1 / _vparams.m, euler, accelNED);

            // We're airborne once net downward acceleration goes below zero
            float netz = accelNED[2] + _wparams.g;

            // If we're airborne, check for low AGL on descent
            if (_airborne) {

                if (agl <= 0 && netz >= 0) {

                    _airborne = false;
                    
                    state.dx = 0;
                    state.dy = 0;
                    state.z = _state.z + agl;
                    state.dz = 0;
                    state.phi = 0;
                    state.dphi = 0;
                    state.theta = 0;
                    state.dtheta = 0;
                    state.dpsi = 0;
                }
            }

            // If we're not airborne, we become airborne when downward
            // acceleration has become negative
            else {
                _airborne = netz < 0;
            }

            float dphi   = _state.dphi;
            float dtheta = _state.dtheta;
            float dpsi   = _state.dpsi;

            float Ix = _vparams.Ix;
            float Iy = _vparams.Iy;
            float Iz = _vparams.Iz;
            float Jr = _vparams.Jr;

            // Once airborne, we can update dynamics
            if (_airborne) {

                // Compute the state derivatives using Equation 12, and integrate them
                // to get the updated state
                state.x      = _state.x + dt * _state.dx;
                state.dx     = _state.dx + dt * accelNED[0];
                state.y      = _state.y + dt * _state.dy;
                state.dy     = _state.dy + dt * accelNED[1];
                state.z      = _state.z + dt * _state.dz;
                state.dz     = _state.dz + dt * netz;
                state.phi    = _state.phi + dt * _state.dphi;
                state.dphi   = _state.dphi + dt * (dpsi * dtheta *(Iy - Iz) / Ix - Jr / Ix * dtheta * omega + u2 / Ix);
                state.theta  = _state.theta + dt * _state.dtheta;
                state.dtheta = _state.dtheta + dt * (-(dpsi * dphi * (Iz - Ix) / Iy + Jr / Iy * dphi * omega + u3 / Iy));
                state.psi    = _state.psi + dt * _state.dpsi;
                state.dpsi   = _state.dpsi + dt * (dtheta * dphi * (Ix - Iy) / Iz + u4 / Iz); 
            }
            else {
                
                // "fly" to agl=0
                state.z = _state.z + (5 * agl) * dt;
            }

            // Maintain state between calls
            memcpy(&_state, &state, sizeof(state_t));
            _time = time;

        } // update

}; // class Dynamics
