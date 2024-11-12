/*
   Proxy for testing MulticopterSim CrazyFlie comms

   Just lifts off to 40cm after enough throttle is applied; no
   other control.

   Copyright(C) 2023 Simon D.Levy

   MIT License
 */

#include <stdio.h>
#include <stdio.h>
#include <stdint.h>

#include "../sockets/TcpServerSocket.hpp"
#include "dynamics/fixedpitch/QuadXBF.hpp"

// Comms
static const char * HOST = "127.0.0.1"; // localhost
static const uint16_t  PORT = 5000;

// Time constant
static const double DELTA_T = 0.001;

// Throttle threshold for liftoff
static const float THROTTLE_THRESHOLD = 0.5;

// PI controller constants
static const double K_P = 4.0;
static const double K_I = 1.0;
static const double K_WINDUP_MAX = 1.0;
static const double Z_TARGET = 0.40;

static Dynamics::vehicle_params_t vparams = {

    // Estimated
    2.E-06, // d torque constant [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // m mass [kg]

    // Estimated
    2,      // Ix [kg*m^2] 
    2,      // Iy [kg*m^2] 
    3,      // Iz [kg*m^2] 
    38E-04, // Jr prop inertial [kg*m^2] 

    15000 // maxrpm
};

static FixedPitchDynamics::fixed_pitch_params_t fparams = {

    // Estimated
    5.E-06, // b force constatnt [F=b*w^2]
    0.350   // l arm length [m]
};

static float constrain(const float val, const float min, const float max)
{
    return val < min ? min : val > max ? max : val;
}

// Altitude PI controller
static float getThrottle(const double z, const double dz)
{
    const auto error = (Z_TARGET - z) - dz;

    static double errorIntegral;

    errorIntegral = constrain(
            errorIntegral + error, -K_WINDUP_MAX, +K_WINDUP_MAX);

    return constrain(K_P * error + K_I * errorIntegral, 0, 1);
}

int main(int argc, char ** argv)
{
    auto server = TcpServerSocket(HOST, PORT, true);

    // Guards socket comms
    auto connected = false;

    // Create quadcopter dynamics model
    auto dynamics = QuadXBFDynamics(vparams, fparams, false); // no auto-land

    // Set up initial conditions
    const double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    printf("Listening for client on %s:%d \n", HOST, PORT);

    // Loop forever, waiting for clients
    for (uint32_t k=0; ; k++) {

        if (connected) {

            const auto vstate = dynamics._vstate;
          
            const double pose[] = {

                vstate.x,
                vstate.y,
                -vstate.z, // NED => ENU
                vstate.phi,
                vstate.theta,
                vstate.psi
            };

            server.sendData((void *)pose, sizeof(pose));

            double joyvals[4] = {};
            server.receiveData(joyvals, sizeof(joyvals));

            float sticks[4] = {
                (float)joyvals[0] / 80,
                (float)joyvals[1] / 31,
                (float)joyvals[2] / 31,
                (float)joyvals[3] / 200,
            };

            static bool airborne;

            if (sticks[0] > THROTTLE_THRESHOLD) {
                airborne = true;
            }

            const auto throttle = airborne ? 
                getThrottle(-vstate.z, -vstate.dz) : 
                0;

            printf("throttle=%3.3f  altitude=%3.3f\n", throttle, -vstate.z);
            
            // Set all motors to same value for now
            const float motors[4] = {throttle, throttle, throttle, throttle};

            // Update dynamics with motor values
            dynamics.update(motors, DELTA_T);

            // Set AGL to arbitrary positive value to avoid kinematic trick
            dynamics.setAgl(1);
        }

        else {

            connected = server.acceptConnection();

        }

    } // while (true)

    return 0;
}
