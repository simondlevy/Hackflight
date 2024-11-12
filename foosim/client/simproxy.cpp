/*
   UDP proxy for testing MulticopterSim socket comms

   Copyright(C) 2019 Simon D.Levy

   MIT License
 */

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
static uint16_t  IMAGE_PORT = 5002;

// Image size
static uint16_t IMAGE_ROWS = 480;
static uint16_t IMAGE_COLS = 640;

// Time constant
static const double DELTA_T = 0.001;

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

int main(int argc, char ** argv)
{
    // Allocate image bytes (rows * cols * rgba)
    uint8_t image[IMAGE_ROWS * IMAGE_COLS * 4];

    memset(image, 0, sizeof(image));

    // Create proxy image with diagonal red stripe
    for (uint16_t j=0; j<IMAGE_ROWS; ++j) {
        for (uint16_t k=0; k<IMAGE_COLS; ++k) {
            uint16_t l = (float)j/IMAGE_ROWS * IMAGE_COLS;
            image[(j*IMAGE_COLS+l)*4] = 255;
        }
    }

    // Create sockets for telemetry out, motors in
    UdpClientSocket telemClient =
        UdpClientSocket(HOST, TELEM_PORT);
    UdpServerSocket motorServer =
        UdpServerSocket(MOTOR_PORT);

    // Create one-way server for images out
    TcpClientSocket imageSocket = TcpClientSocket(HOST, IMAGE_PORT);

    // Create quadcopter dynamics model
    QuadXBFDynamics dynamics =
        QuadXBFDynamics(vparams, fparams, false); // no auto-land

    // Set up initial conditions
    double time = 0;
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    // Open image socket's connection to host
    imageSocket.openConnection();

    // Loop forever, communicating with server
    while (true) {

        // To be sent to client
        double telemetry[17] = {0};

        // First value is time
        telemetry[0] = time;

        // Next 12 values are 12D state vector
        telemetry[1] = dynamics._vstate.x;
        telemetry[2] = dynamics._vstate.dx;
        telemetry[3] = dynamics._vstate.y;
        telemetry[4] = dynamics._vstate.dy;
        telemetry[5] = dynamics._vstate.z;
        telemetry[6] = dynamics._vstate.dz;
        telemetry[7] = dynamics._vstate.phi;
        telemetry[8] = dynamics._vstate.dphi;
        telemetry[9] = dynamics._vstate.theta;
        telemetry[10] = dynamics._vstate.dtheta;
        telemetry[11] = dynamics._vstate.psi;
        telemetry[12] = dynamics._vstate.dpsi;

        // Last four values are receiver demands
        telemetry[13] = 0.1;
        telemetry[14] = 0.2;
        telemetry[15] = 0.3;
        telemetry[16] = 0.4;

        // Send telemetry data
        telemClient.sendData(telemetry, sizeof(telemetry));

        // Send image data
        // imageSocket.sendData(image, sizeof(image));

        // Get incoming motor values
        float motorvals[4] = {};
        motorServer.receiveData(motorvals, sizeof(motorvals));

        printf("t=%05f   m=%f %f %f %f  z=%+3.3f\n", 
                time,
                motorvals[0],
                motorvals[1],
                motorvals[2],
                motorvals[3],
                dynamics._vstate.z);

        // Update dynamics with motor values
        dynamics.update(motorvals, DELTA_T);

        // Set AGL to arbitrary positive value to avoid kinematic trick
        dynamics.setAgl(1);

        time += DELTA_T;
    }


    return 0;
}
