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
#include <math.h>
#include <sys/time.h>

class Dynamics {

    public:

        // arbitrary; avoids dynamic allocation
        static const uint8_t MAX_ROTORS = 20; 

    // private:
    public:

        // state vector (see Eqn. 11)
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

        } vehicle_state_t;

        vehicle_state_t state;

        typedef struct {

            double g;  // gravitational constant
            double rho;  // air density

        } world_params_t; 

        world_params_t EARTH_PARAMS = { 
            9.80665,  // g graviational constant
            1.225 // rho air density 
        };

        bool _autoland; // support fly-to-zero-AGL

    public:

        /**
         *  Vehicle parameters
         */
        typedef struct {

            double d;  // drag coefficient [T=d*w^2]
            double m;  // mass [kg]
            double Ix; // [kg*m^2] 
            double Iy; // [kg*m^2] 
            double Iz; // [kg*m^2] 
            double Jr; // rotor inertial [kg*m^2] 

        } vehicle_params_t; 

        /**
         * Position map for state vector
         */
        enum {
            STATE_X,
            STATE_DX,
            STATE_Y,
            STATE_DY,
            STATE_Z,
            STATE_DZ,
            STATE_PHI,
            STATE_DPHI,
            STATE_THETA,
            STATE_DTHETA,
            STATE_PSI,
            STATE_DPSI,
            STATE_SIZE
        };

    protected:

        vehicle_params_t _vparams;
        world_params_t _wparams;

        vehicle_state_t state_deriv;

        Dynamics(
                const uint8_t actuatorCount,
                const vehicle_params_t & vparams,
                const bool autoland=true)
        {
            _autoland = autoland; 

            _actuatorCount = actuatorCount;

            // can be overridden for thrust-vectoring
            _rotorCount = actuatorCount; 

            memcpy(&_vparams, &vparams, sizeof(vehicle_params_t));

            // Default to Earth params (can be overridden by setWorldParams())
            memcpy(&_wparams, &EARTH_PARAMS, sizeof(world_params_t));

            memset(&state, 0, sizeof(state));
        }        

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

        // Height above ground, set by kinematics
        double _agl = 0;


        // quad, hexa, octo, etc.
        uint8_t _rotorCount = 0;

        // For coaxials we have five omegas: two rotors, plus collective
        // pitch, cyclic roll, and cyclic pitch.  For thrust vectoring, we have
        // four omegas: two rotors and two servos.
        // For standard multirotors (e.g., quadcopter), actuatorCount =
        // rotorCount.
        uint8_t _actuatorCount = 0;

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


    public:

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
            bodyZToInertial(-_wparams.g, rotation, _inertialAccel);

            // We usuall start on ground, but can start in air for testing
            _airborne = airborne;
        }


        /**
         * Sets height above ground level (AGL).
         * This method can be called by the kinematic visualization.
         */
        void setAgl(const double agl)
        {
            _agl = agl;
        }

        // Different for each vehicle

        virtual int8_t getRotorDirection(const uint8_t i) = 0;

        virtual double getThrustCoefficient(double * omegas) = 0;

        virtual void computeRollAndPitch(double * omegas,
                                         double * omegas2,
                                         double & roll,
                                         double & pitch) = 0;

        /**
         * Gets actuator count set by constructor.
         * @return actuator count
         */
        uint8_t actuatorCount(void)
        {
            return _actuatorCount;
        }

        /**
         * Gets rotor count set by constructor.
         * @return rotor count
         */
        uint8_t rotorCount(void)
        {
            return _rotorCount;
        }


        /**
          * Sets world parameters (currently just gravity and air density)
          */
        void setWorldParams(const double g, const double rho)
        {
            _wparams.g = g;
            _wparams.rho = rho;
        }

        /**
         * Updates state.
         *
         * @param motor spins in radians per second
         */
        void update(const float * fomegas) 
        {
            // Get deltaT from clock time
            static double _time_prev;
            struct timeval timeval = {};
            gettimeofday(&timeval, NULL);
            const double time_curr = timeval.tv_sec + (double)timeval.tv_usec / 1e6;
            const auto dt = time_curr - _time_prev;
            _time_prev = time_curr;

            static uint64_t count;
            if (count++ % 10000 == 0) {
                printf("dt = %f\n", dt);
            }

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
                omegas2[i] = _wparams.rho * omegas[i] * omegas[i]; 

                // Thrust coefficient is constant for fixed-pitch rotors,
                // variable for collective-pitch
                u1 += getThrustCoefficient(omegas) * omegas2[i];                  

                // Newton's Third Law (action/reaction) tells us that yaw is
                // opposite to net rotor spin
                u4 += _vparams.d * omegas2[i] * -getRotorDirection(i);
                omega += omegas[i] * -getRotorDirection(i);
            }
            
            // Compute roll, pitch, yaw forces (different method for
            // fixed-pitch blades vs. variable-pitch)
            double u2 = 0, u3 = 0;
            computeRollAndPitch(omegas, omegas2, u2, u3);

            // ----------------------------------------------------------------

            // Use the current Euler angles to rotate the orthogonal thrust
            // vector into the inertial frame
            double euler[3] = {state.phi, state.theta, state.psi};
            double accelNED[3] = {};
            bodyZToInertial(u1 / _vparams.m, euler, accelNED);

            // We're airborne once net downward acceleration goes below zero
            double netz = accelNED[2] + _wparams.g;

            netz = 1e-10;

            // If we're airborne, check for low AGL on descent
            if (_airborne) {

                if (_agl <= 0 && netz >= 0) {

                    _airborne = false;

                    state.dx = 0;
                    state.dy = 0;
                    state.dz = 0;
                    state.phi = 0;
                    state.dphi = 0;
                    state.theta = 0;
                    state.dtheta = 0;
                    state.dpsi = 0;

                    state.z += _agl;
                }
            }

            // If we're not airborne, we become airborne when upward
            // acceleration has become positive
            else {
                _airborne = netz > 0;
            }

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
            else if (_autoland) {
                //"fly" to agl=0
                state.z += 5 * _agl * dt;
            }

            // XXX
            //vstate.z = -1;

        } // update

        double getStateX(void)
        {
            return state.x;
        }

        double getStateDx(void)
        {
            return state.dx;
        }

        double getStateY(void)
        {
            return state.y;
        }

        double getStateDy(void)
        {
            return state.dy;
        }

        double getStateZ(void)
        {
            return -state.z; // NED => ENU
        }

        double getStateDz(void)
        {
            return -state.dz; // NED => ENU
        }

        double getStatePhi(void)
        {
            return state.phi;
        }

        double getStateDphi(void)
        {
            return state.dphi;
        }

        double getStateTheta(void)
        {
            return state.theta;
        }

        double getStateDtheta(void)
        {
            return state.dtheta;
        }

        double getStatePsi(void)
        {
            return state.psi;
        }

        double getStateDpsi(void)
        {
            return state.dpsi;
        }

}; // class Dynamics
