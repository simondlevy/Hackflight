#include <stdio.h>
#include <stdio.h>
#include <stdint.h>

#include "../sockets/UdpClientSocket.hpp"
#include "../sockets/UdpServerSocket.hpp"
#include "../sockets/TcpClientSocket.hpp"
#include "dynamics/fixedpitch/QuadXBF.hpp"

// Comms
static const char * HOST = "127.0.0.1"; // localhost
static uint16_t  MOTOR_PORT = 5000;
static uint16_t  TELEM_PORT = 5001;

// Time constant
static const double DT = 2.0e-5;

// Vehicle constants

static Dynamics::vehicle_params_t vparams = {

    2.e-06,  // drag coefficient [T=d*w^2]
    0.05,    // m mass [kg]
    2,       // Ix [kg*m^2] 
    2,       // Iy [kg*m^2] 
    3,       // Iz [kg*m^2] 
    3.8e-03  // Jr prop inertial [kg*m^2] 
};

static FixedPitchDynamics::fixed_pitch_params_t fparams = {

    // Estimated
    3.275e-5, // b force constatnt [F=b*w^2]
    0.03    // l arm length [m]
};

int main(int argc, char ** argv)
{
    // Create sockets for telemetry out, motors in
    UdpClientSocket telemClient =
        UdpClientSocket(HOST, TELEM_PORT);
    UdpServerSocket motorServer =
        UdpServerSocket(MOTOR_PORT);

    // Create quadcopter dynamics model
    QuadXBFDynamics dynamics =
        QuadXBFDynamics(vparams, fparams, false); // no auto-land

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    // Loop forever, communicating with server
    for (uint64_t k=0; ; ++k) {

        // To be sent to client
        double telemetry[17] = {0};

        const double time = k * DT;

        // First value is time
        telemetry[0] = time;

        // Next 12 values are 12D state vector
        telemetry[1] = dynamics.state.x;
        telemetry[2] = dynamics.state.dx;
        telemetry[3] = dynamics.state.y;
        telemetry[4] = dynamics.state.dy;
        telemetry[5] = dynamics.state.z;
        telemetry[6] = dynamics.state.dz;
        telemetry[7] = dynamics.state.phi;
        telemetry[8] = dynamics.state.dphi;
        telemetry[9] = dynamics.state.theta;
        telemetry[10] = dynamics.state.dtheta;
        telemetry[11] = dynamics.state.psi;
        telemetry[12] = dynamics.state.dpsi;

        // Last four values are receiver demands
        telemetry[13] = 0.1;
        telemetry[14] = 0.2;
        telemetry[15] = 0.3;
        telemetry[16] = 0.4;

        // Send telemetry data
        telemClient.sendData(telemetry, sizeof(telemetry));

        // Get incoming motor values
        float motorvals[4] = {};
        motorServer.receiveData(motorvals, sizeof(motorvals));

        printf("t=%05f   m=%f %f %f %f  z=%+3.3f\n", 
                time,
                motorvals[0],
                motorvals[1],
                motorvals[2],
                motorvals[3],
                dynamics.state.z);

        // Update dynamics with motor values
        dynamics.update(motorvals, DT);

        // Set AGL to arbitrary positive value to avoid kinematic trick
        dynamics.setAgl(1);
    }

    return 0;
}
