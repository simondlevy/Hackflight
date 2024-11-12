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

#define WIN32_LEAN_AND_MEAN

#define _USE_MATH_DEFINES
#include <math.h>

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

        vehicle_state_t _vstate;

        typedef struct {

            double g;  // gravitational constant
            double rho;  // air density

        } world_params_t; 

        /* XXX static constexpr*/ world_params_t EARTH_PARAMS = { 
            9.80665,  // g graviational constant
            1.225 // rho air density 
        };

        bool _autoland; // support fly-to-zero-AGL

        double _capSpeed(const double speed)
        {
            const auto cap = _vparams.maxspeed;

            return speed < -cap ? -cap : speed > cap ? cap : speed;
        }

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
            uint16_t maxrpm; // maxrpm
            double maxspeed; // [m/s]

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

        vehicle_state_t _vstate_deriv;

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

            memset(&_vstate, 0, sizeof(_vstate));
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

        // For coaxials we have five actuators: two rotors, plus collective
        // pitch, cyclic roll, and cyclic pitch.  For thrust vectoring, we have
        // four actuators: two rotors and two servos.
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
            double phidot = _vstate.dphi;
            double thedot = _vstate.dtheta;
            double psidot = _vstate.dpsi;

            double Ix = _vparams.Ix;
            double Iy = _vparams.Iy;
            double Iz = _vparams.Iz;
            double Jr = _vparams.Jr;

            // x'
            _vstate_deriv.x = _vstate.dx;
            
            // x''
            _vstate_deriv.dx = accelNED[0];

            // y'
            _vstate_deriv.y = _vstate.dy;

            // y''
            _vstate_deriv.dy = accelNED[1];

            // z'
            _vstate_deriv.z = _vstate.dz;

            // z''
            _vstate_deriv.dz = netz;

            // phi'
            _vstate_deriv.phi = phidot;

            // phi''
            _vstate_deriv.dphi = psidot * thedot * (Iy - Iz) / Ix - Jr / 
                Ix * thedot * omega + u2 / Ix;

            // theta'
            _vstate_deriv.theta = thedot;

            // theta''
            _vstate_deriv.dtheta = -(psidot * phidot * (Iz - Ix) / Iy + Jr / 
                    Iy * phidot * omega + u3 / Iy);

            // psi'
            _vstate_deriv.psi = psidot;

            // psi''
            _vstate_deriv.dpsi = thedot * phidot * (Ix - Iy) / Iz + u4 / Iz;
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
            memset(&_vstate, 0, sizeof(_vstate));

            _vstate.phi   = rotation[0];
            _vstate.theta = rotation[1];
            _vstate.psi   = rotation[2];

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

        virtual double getThrustCoefficient(double * actuators) = 0;

        virtual void computeRollAndPitch(double * actuators,
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
         * @param actuators values in interval [0,1] (rotors) or [-0.5,+0.5]
                  (servos)
         * @param dt time in seconds since previous update
         */
        void update(const float * factuators, const double dt) 
        {
            // Convert actuator values to double-precision for consistency
            double actuators[10];
            for (auto k=0; k<_rotorCount; ++k) {
                actuators[k] = factuators[k];
            }

            // Implement Equation 6 -------------------------------------------

            // Radians per second of rotors, and squared radians per second
            double omegas[MAX_ROTORS] = {};
            double omegas2[MAX_ROTORS] = {};

            double u1 = 0, u4 = 0, omega = 0;
            for (unsigned int i = 0; i < _rotorCount; ++i) {

                // Convert fractional speed to radians per second
                omegas[i] = actuators[i] * _vparams.maxrpm * M_PI / 30;  

                // Thrust is squared rad/sec scaled by air density
                omegas2[i] = _wparams.rho * omegas[i] * omegas[i]; 

                // Thrust coefficient is constant for fixed-pitch rotors,
                // variable for collective-pitch
                u1 += getThrustCoefficient(actuators) * omegas2[i];                  

                // Newton's Third Law (action/reaction) tells us that yaw is
                // opposite to net rotor spin
                u4 += _vparams.d * omegas2[i] * -getRotorDirection(i);
                omega += omegas[i] * -getRotorDirection(i);
            }
            
            // Compute roll, pitch, yaw forces (different method for
            // fixed-pitch blades vs. variable-pitch)
            double u2 = 0, u3 = 0;
            computeRollAndPitch(actuators, omegas2, u2, u3);

            // ----------------------------------------------------------------

            // Use the current Euler angles to rotate the orthogonal thrust
            // vector into the inertial frame.  Negate to use NED.
            double euler[3] = {_vstate.phi, _vstate.theta, _vstate.psi};
            double accelNED[3] = {};
            bodyZToInertial(-u1 / _vparams.m, euler, accelNED);

            // We're airborne once net downward acceleration goes below zero
            double netz = accelNED[2] + _wparams.g;

            // If we're airborne, check for low AGL on descent
            if (_airborne) {

                if (_agl <= 0 && netz >= 0) {

                    _airborne = false;

                    _vstate.dx = 0;
                    _vstate.dy = 0;
                    _vstate.dz = 0;
                    _vstate.phi = 0;
                    _vstate.dphi = 0;
                    _vstate.theta = 0;
                    _vstate.dtheta = 0;
                    _vstate.dpsi = 0;

                    _vstate.z += _agl;
                }
            }

            // If we're not airborne, we become airborne when downward
            // acceleration has become negative
            else {
                _airborne = netz < 0;
            }

            // Once airborne, we can update dynamics
            if (_airborne) {

                // Compute the state derivatives using Equation 12
                computeStateDerivative(accelNED, netz, omega, u2, u3, u4);

                // Compute state as first temporal integral of first temporal
                // derivative
                _vstate.x += dt * _vstate_deriv.x;
                _vstate.dx += dt * _vstate_deriv.dx;
                _vstate.y += dt * _vstate_deriv.y;
                _vstate.dy += dt * _vstate_deriv.dy;
                _vstate.z += dt * _vstate_deriv.z;
                _vstate.dz += dt * _vstate_deriv.dz;
                _vstate.phi += dt * _vstate_deriv.phi;
                _vstate.dphi += dt * _vstate_deriv.dphi;
                _vstate.theta += dt * _vstate_deriv.theta;
                _vstate.dtheta += dt * _vstate_deriv.dtheta;
                _vstate.psi += dt * _vstate_deriv.psi;
                _vstate.dpsi += dt * _vstate_deriv.dpsi;

                // Cap dx, dy by maximum speed
                _vstate.dx = _capSpeed(_vstate.dx);
                _vstate.dy = _capSpeed(_vstate.dy);

                // Once airborne, inertial-frame acceleration is same as NED
                // acceleration
                _inertialAccel[0] = accelNED[0];
                _inertialAccel[1] = accelNED[1];
                _inertialAccel[2] = accelNED[2];
            }
            else if (_autoland) {
                //"fly" to agl=0
                _vstate.z += 5 * _agl * dt;
            }

            // XXX
            //vstate.z = -1;

        } // update

        double getStateX(void)
        {
            return _vstate.x;
        }

        double getStateDx(void)
        {
            return _vstate.dx;
        }

        double getStateY(void)
        {
            return _vstate.y;
        }

        double getStateDy(void)
        {
            return _vstate.dy;
        }

        double getStateZ(void)
        {
            return -_vstate.z; // NED => ENU
        }

        double getStateDz(void)
        {
            return -_vstate.dz; // NED => ENU
        }

        double getStatePhi(void)
        {
            return _vstate.phi;
        }

        double getStateDphi(void)
        {
            return _vstate.dphi;
        }

        double getStateTheta(void)
        {
            return _vstate.theta;
        }

        double getStateDtheta(void)
        {
            return _vstate.dtheta;
        }

        double getStatePsi(void)
        {
            return _vstate.psi;
        }

        double getStateDpsi(void)
        {
            return _vstate.dpsi;
        }

}; // class Dynamics
