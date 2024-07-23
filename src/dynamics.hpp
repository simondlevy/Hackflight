/*
Multirotor Dynamics class

Should work for any simulator, vehicle, or operating system

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

Copyright (C) 2021 Simon D. Levy, Alex Sender

MIT License
*/

#pragma once

#include <math.h>
#include <string.h>

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
        float _dxdt[12];

        // Flight status
        status_e _status;

        float _inertialAccel[3];

    public:

        // Indices into state vector
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
            STATE_PSI_DOT

        } state_e;

        // State vector
        float x[12];

        Dynamics(const vehicle_params_t & params, const float dt)
        {
            memcpy(&_params, &params, sizeof(vehicle_params_t));

            _dt = dt;

            // Always start at location (0,0,0) with zero velocities
            memset(x, 0, sizeof(x)); 
            memset(_dxdt, 0, sizeof(_dxdt));

            // Start on ground
            _status = STATUS_LANDED;

            // Initialize inertial frame acceleration in NED coordinates
            bodyZToInertial(-G, 0, 0, 0, _inertialAccel);
        }

        /*
           Implements Equations 6 and 12 from Bouabdallah et al. (2004)
           @param omegas motor speeds in radians per second
         */

        void setMotors(
                const float omega0,
                const float omega1,
                const float omega2,
                const float omega3)
        {
            // Compute individual motor thrusts as air density times square of
            // motor speed
            const float omega0_2 = sqr(omega0);
            const float omega1_2 = sqr(omega1);
            const float omega2_2 = sqr(omega2);
            const float omega3_2 = sqr(omega3);

            // Compute overall thrust, plus roll and pitch
            const auto U1 =
                _params.B * (omega0_2 + omega1_2 + omega2_2 + omega3_2);
            const auto U2 = _params.L * _params.B * 
                u2(omega0_2, omega1_2, omega2_2, omega3_2);
            const auto U3 = _params.L * _params.B * 
                u3(omega0_2, omega1_2, omega2_2, omega3_2);

            // Compute yaw torque
            const auto U4 = _params.D * 
                u4(omega0_2, omega1_2, omega2_2, omega3_2);

            // Ignore Omega ("disturbance") part of Equation 6 for now
            const float Omega = 0;

            // Use the current Euler angles to rotate the orthogonal thrust
            // vector into the inertial frame.  
            float accelENU[3] = {};
            bodyZToInertial(U1 / _params.M, x[6], x[8], x[10], accelENU);

            // Compute net vertical acceleration by subtracting gravity
            const auto netz = accelENU[2] - G;

            // If we're not airborne, we become airborne when upward acceleration
            // has become positive
            if (_status == STATUS_LANDED) {

                if (netz > 0) {
                    _status = STATUS_AIRBORNE;
                }
            }

            // Once airborne, we can update dynamics
            else if (_status == STATUS_AIRBORNE) {

                 // If we've descended to the ground
                if (x[STATE_Z] <= 0 and x[STATE_Z_DOT] <= 0) {
                
                    // Big angles indicate a crash
                    const auto phi = x[STATE_PHI];
                    const auto velx = x[STATE_Y_DOT];
                    const auto vely = x[STATE_Z_DOT];
                    if ((vely > LANDING_VEL_Y ||
                       fabs(velx) > LANDING_VEL_X ||
                       fabs(phi) > LANDING_ANGLE)) {
                        _status = STATUS_CRASHED;
                    }
                    else {
                        _status = STATUS_LANDED;
                    }

                    x[STATE_Z] = 0;
                    x[STATE_Z_DOT] = 0;
                }

                // Compute the state derivatives using Equation 12
                computeStateDerivative(accelENU, netz, U2, U3, U4, Omega);

                // Compute state as first temporal integral of first temporal
                // derivative
                for (int k=0; k<12; ++k) {
                    x[k] += _dt * _dxdt[k];
                }

                // Once airborne, inertial-frame acceleration is same as NED
                // acceleration
                memcpy(_inertialAccel, accelENU, sizeof(accelENU));
        }
    }

        /*
        Sets the vehicle position to the values specified in a sequence
        */
        void setPosition(const float x, const float y, const float z)
        {
            memset(this->x, 0, sizeof(x));

            this->x[STATE_X] = x;
            this->x[STATE_Y] = y;
            this->x[STATE_Z] = z;

            _status = z > 0 ? STATUS_AIRBORNE : STATUS_LANDED;
        }

    private:

        /*
           Implements Equation 12 computing temporal first derivative of state.
           Should fill _dxdx[0..11] with appropriate values.
         */
        void computeStateDerivative(
                const float accelENU[3],
                const float netz,
                const float U2,
                const float U3,
                const float U4,
                const float Omega)
        {
            const auto phidot = x[STATE_PHI_DOT];
            const auto thedot = x[STATE_THETA_DOT];
            const auto psidot = x[STATE_PSI_DOT];

            _dxdt[STATE_X] = x[STATE_X_DOT];

            _dxdt[STATE_X_DOT] = accelENU[0];

            _dxdt[STATE_Y] = x[STATE_Y_DOT];

            _dxdt[STATE_Y_DOT] = accelENU[1];

            _dxdt[STATE_Z] = x[STATE_Z_DOT];

            _dxdt[STATE_Z_DOT] = netz;

            _dxdt[STATE_PHI] = phidot;

            _dxdt[STATE_PHI_DOT] = (
                    psidot*thedot*(_params.Iy-_params.Iz) /
                    _params.Ix-_params.Jr /
                    _params.Ix*thedot*Omega + U2 / _params.Ix);

            _dxdt[STATE_PSI] = psidot;

            _dxdt[STATE_PSI_DOT] = (
                    thedot*phidot*(_params.Ix-_params.Iy)/_params.Iz +
                    U4/_params.Iz);
        }

        static float u2(
                const float o0, const float o1, const float o2, const float o3)
        {
            /*
               roll right
             */

            return (o2 + o3) - (o0 + o1);
        }

        static float u3(
                const float o0, const float o1, const float o2, const float o3)
        {
            /*
               pitch forward
             */

            return (o1 + o2) - (o0 + o3);
        }

        static float u4(
                const float o0, const float o1, const float o2, const float o3)
        {
            /*
               yaw cw
             */

            return (o0 + o2) - (o1 + o3);
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
                float inertial[3])
        {

            float cph=0;
            float cth=0;
            float cps=0;
            float sph=0;
            float sth=0;
            float sps=0;

            sincos(phi, theta, psi, cph, cth, cps, sph, sth, sps);

            inertial[0] = bodyZ * sph*sps+cph*cps*sth;
            inertial[1] = bodyZ * cph*sps*sth-cps*sph;
            inertial[2] = bodyZ * cph*cth;
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
